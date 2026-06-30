#include "adc_dma.hpp"
#include <cstring>
#include <esp_attr.h>
#include <esp_log.h>

#define CHECK_RET(x) do { esp_err_t _err = (x); if (_err != ESP_OK) return _err; } while(0)

static const char *TAG = "AdcDmaDriver";

EspAdcDmaDriver::EspAdcDmaDriver()
    : _handle(nullptr), _is_initialized(false), _is_running(false), _latest_data{} {
    for (int i = 0; i < NUM_SENSORS; i++) {
        _cali_handle[i] = nullptr;
    }
}

EspAdcDmaDriver::~EspAdcDmaDriver() {
    stop();
    if (_is_initialized) {
        adc_continuous_deinit(_handle);
        for (int i = 0; i < NUM_SENSORS; i++) {
            _deinit_calibration(_cali_handle[i]);
        }
    }
}

void EspAdcDmaDriver::_init_calibration(adc_unit_t unit, adc_channel_t channel,
                                        adc_atten_t atten,
                                        adc_cali_handle_t *out_handle) {
    adc_cali_line_fitting_config_t cali_config = {};
    cali_config.unit_id = unit;
    cali_config.atten = atten;
    cali_config.bitwidth = ADC_BITWIDTH_12;
#if CONFIG_IDF_TARGET_ESP32
    cali_config.default_vref = 1100;
#endif

    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, out_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration OK for channel %d", channel);
    } else if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration missing for channel %d", channel);
    } else {
        ESP_LOGE(TAG, "Calibration fail for channel %d", channel);
    }
}

void EspAdcDmaDriver::_deinit_calibration(adc_cali_handle_t handle) {
    if (handle) {
        adc_cali_delete_scheme_line_fitting(handle);
    }
}

esp_err_t EspAdcDmaDriver::init(const adc_dma_config_t &config) {
    if (_is_initialized) return ESP_ERR_INVALID_STATE;
    _config = config;

    adc_continuous_handle_cfg_t handle_cfg = {};
    handle_cfg.max_store_buf_size = _config.dma_frame_size * 4;
    handle_cfg.conv_frame_size = _config.dma_frame_size;
    CHECK_RET(adc_continuous_new_handle(&handle_cfg, &_handle));

    adc_continuous_config_t adc_cfg = {};
    adc_cfg.sample_freq_hz = _config.sample_freq_hz;
    adc_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    adc_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;

    adc_digi_pattern_config_t adc_pattern[NUM_SENSORS] = {};
    for (int i = 0; i < NUM_SENSORS; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_12;
        adc_pattern[i].channel = _config.channel_map[i];
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = ADC_BITWIDTH_12;
        _init_calibration(ADC_UNIT_1, _config.channel_map[i], ADC_ATTEN_DB_12, &_cali_handle[i]);
    }
    adc_cfg.pattern_num = NUM_SENSORS;
    adc_cfg.adc_pattern = adc_pattern;

    CHECK_RET(adc_continuous_config(_handle, &adc_cfg));

    adc_continuous_evt_cbs_t cbs = {};
    cbs.on_pool_ovf = _on_pool_ovf;
    CHECK_RET(adc_continuous_register_event_callbacks(_handle, &cbs, this));

    _is_initialized = true;
    return ESP_OK;
}

bool IRAM_ATTR EspAdcDmaDriver::_on_pool_ovf(
    adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata,
    void *user_data) {
    return false;
}

esp_err_t EspAdcDmaDriver::start() {
    if (!_is_initialized || _is_running) return ESP_ERR_INVALID_STATE;

    CHECK_RET(adc_continuous_start(_handle));

    _is_running = true;
    return ESP_OK;
}

esp_err_t EspAdcDmaDriver::stop() {
    if (!_is_running) return ESP_ERR_INVALID_STATE;

    _is_running = false;
    CHECK_RET(adc_continuous_stop(_handle));
    return ESP_OK;
}

esp_err_t EspAdcDmaDriver::process_dma_events(TickType_t timeout) {
    if (!_is_running) return ESP_ERR_INVALID_STATE;

    uint8_t result_buf[256];
    uint32_t ret_num = 0;
    adc_sensor_data_t local_frame = {};
    int count_found = 0;

    uint32_t timeout_ms = (timeout == portMAX_DELAY) ? portMAX_DELAY : (timeout * portTICK_PERIOD_MS);
    esp_err_t ret = adc_continuous_read(_handle, result_buf, sizeof(result_buf), &ret_num, timeout_ms);

    if (ret == ESP_OK) {
        for (uint32_t i = 0; i < ret_num; i += sizeof(adc_digi_output_data_t)) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result_buf[i];
            uint32_t chan_num = p->type1.channel;
            uint32_t data = p->type1.data;

            for (int s = 0; s < NUM_SENSORS; s++) {
                if (_config.channel_map[s] == chan_num) {
                    local_frame.raw[s] = data;
                    if (_cali_handle[s]) {
                        adc_cali_raw_to_voltage(_cali_handle[s], data, &local_frame.voltage_mv[s]);
                    } else {
                        local_frame.voltage_mv[s] = -1;
                    }
                    count_found++;
                    break;
                }
            }

            if (count_found >= NUM_SENSORS) {
                portENTER_CRITICAL(&_spinlock);
                _latest_data = local_frame;
                portEXIT_CRITICAL(&_spinlock);
                count_found = 0;
            }
        }
    } else if (ret == ESP_ERR_TIMEOUT) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return ret;
}

void EspAdcDmaDriver::read_sensor_data(adc_sensor_data_t *out_data) {
    if (!out_data) return;

    portENTER_CRITICAL(&_spinlock);
    *out_data = _latest_data;
    portEXIT_CRITICAL(&_spinlock);
}
