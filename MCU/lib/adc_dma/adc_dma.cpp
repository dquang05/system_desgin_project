#include "adc_dma.hpp"
#include <cstring>

EspAdcDmaDriver::EspAdcDmaDriver() : 
    _handle(nullptr),
    _is_initialized(false),
    _is_running(false) 
{
    std::memset(&_config, 0, sizeof(adc_dma_config_t));
    std::memset(&_latest_data, 0, sizeof(adc_sensor_data_t));
}

EspAdcDmaDriver::~EspAdcDmaDriver() {
    stop();
    if (_is_initialized && _handle != nullptr) {
        adc_continuous_deinit(_handle);
    }
}

bool IRAM_ATTR EspAdcDmaDriver::_on_pool_ovf(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    return false;
}

esp_err_t EspAdcDmaDriver::init(const adc_dma_config_t& config) {
    if (_is_initialized) return ESP_ERR_INVALID_STATE;
    _config = config;

    adc_continuous_handle_cfg_t handle_cfg = {};
    handle_cfg.max_store_buf_size = 1024;
    handle_cfg.conv_frame_size = _config.dma_frame_size;
    
    esp_err_t ret = adc_continuous_new_handle(&handle_cfg, &_handle);
    if (ret != ESP_OK) return ret;

    adc_continuous_config_t cont_cfg = {};
    cont_cfg.pattern_num = NUM_SENSORS;
    
    adc_digi_pattern_config_t pattern[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
        pattern[i].atten = ADC_ATTEN_DB_12; 
        pattern[i].channel = _config.channel_map[i];
        pattern[i].unit = ADC_UNIT_1;
        pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    cont_cfg.adc_pattern = pattern;
    cont_cfg.sample_freq_hz = _config.sample_freq_hz;
    cont_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;

    ret = adc_continuous_config(_handle, &cont_cfg);
    if (ret != ESP_OK) {
        adc_continuous_deinit(_handle);
        return ret;
    }

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = nullptr,
        .on_pool_ovf = _on_pool_ovf
    };
    ret = adc_continuous_register_event_callbacks(_handle, &cbs, this);
    if (ret != ESP_OK) {
        adc_continuous_deinit(_handle);
        return ret;
    }

    _is_initialized = true;
    return ESP_OK;
}

esp_err_t EspAdcDmaDriver::start() {
    if (!_is_initialized || _is_running) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = adc_continuous_start(_handle);
    if (ret == ESP_OK) {
        _is_running = true;
    }
    return ret;
}

esp_err_t EspAdcDmaDriver::stop() {
    if (!_is_running) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = adc_continuous_stop(_handle);
    if (ret == ESP_OK) {
        _is_running = false;
    }
    return ret;
}

esp_err_t EspAdcDmaDriver::process_dma_events(TickType_t timeout) {
    if (!_is_running) return ESP_ERR_INVALID_STATE;

    uint8_t result_buf[256]; 
    uint32_t out_length = 0;
    
    uint32_t timeout_ms;
    if (timeout == portMAX_DELAY) {
        timeout_ms = -1; // portMAX_DELAY maps to HAL_MAX_DELAY
    } else {
        timeout_ms = timeout * portTICK_PERIOD_MS;
    }

    esp_err_t ret = adc_continuous_read(_handle, result_buf, sizeof(result_buf), &out_length, timeout_ms);
    if (ret == ESP_ERR_TIMEOUT) {
        vTaskDelay(pdMS_TO_TICKS(10));
        return ret;
    } else if (ret != ESP_OK) {
        return ret;
    }

    adc_sensor_data_t local_frame;
    bool updated[NUM_SENSORS] = {false};
    
    for (uint32_t i = 0; i < out_length; i += SOC_ADC_DIGI_RESULT_BYTES) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result_buf[i];
        uint32_t chan_num = p->type1.channel;
        uint32_t data = p->type1.data;
        
        for (int s = 0; s < NUM_SENSORS; s++) {
            if (_config.channel_map[s] == chan_num) {
                local_frame.raw[s] = data;
                updated[s] = true;
                break;
            }
        }
    }

    portENTER_CRITICAL(&_spinlock);
    for (int s = 0; s < NUM_SENSORS; s++) {
        if (updated[s]) {
            _latest_data.raw[s] = local_frame.raw[s];
        }
    }
    portEXIT_CRITICAL(&_spinlock);

    return ESP_OK;
}

void EspAdcDmaDriver::read_sensor_data(adc_sensor_data_t* out_data) {
    if (out_data == nullptr) return;
    
    portENTER_CRITICAL(&_spinlock);
    *out_data = _latest_data;
    portEXIT_CRITICAL(&_spinlock);
}

// RISK REVIEW:
// - Caller Responsibilities: The caller must execute process_dma_events() continuously within a dedicated or appropriately scheduled high-priority task. It is the caller's duty to provide an adequate tick timeout value and handle ESP_ERR_TIMEOUT or other ESP-IDF errors gracefully.
// - Data Dropping Risks: If the task processing rate (the caller loop) falls behind the configured sample_freq_hz, the internal 1024-byte DMA buffer will overflow. The _on_pool_ovf callback will trigger but return false. As a result, older ADC data will be overwritten and permanently lost, causing the AMR to read stale or missing patterns over critical path lines.
