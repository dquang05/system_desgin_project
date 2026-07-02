#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_adc/adc_continuous.h>
#include <esp_err.h>

#define PIN_ADC_SENSOR_01 ADC_CHANNEL_0 // GPIO36 (VP)
#define PIN_ADC_SENSOR_02 ADC_CHANNEL_3 // GPIO39 (VN)
#define PIN_ADC_SENSOR_03 ADC_CHANNEL_4 // GPIO32
#define PIN_ADC_SENSOR_04 ADC_CHANNEL_5 // GPIO33
#define PIN_ADC_SENSOR_05 ADC_CHANNEL_6 // GPIO34

enum SensorId {
    SENSOR_01 = 0,
    SENSOR_02,
    SENSOR_03,
    SENSOR_04,
    SENSOR_05,
    NUM_SENSORS
};

struct adc_dma_config_t {
    adc_channel_t channel_map[NUM_SENSORS];
    uint32_t sample_freq_hz;
    size_t dma_frame_size;
};

struct adc_sensor_data_t {
    uint32_t raw[NUM_SENSORS];
};

class EspAdcDmaDriver {
public:
    EspAdcDmaDriver();
    ~EspAdcDmaDriver();

    esp_err_t init(const adc_dma_config_t& config);
    esp_err_t start();
    esp_err_t stop();
    esp_err_t process_dma_events(TickType_t timeout);
    void read_sensor_data(adc_sensor_data_t* out_data);

private:
    adc_dma_config_t _config;
    adc_continuous_handle_t _handle;
    bool _is_initialized;
    bool _is_running;
    portMUX_TYPE _spinlock = portMUX_INITIALIZER_UNLOCKED;
    adc_sensor_data_t _latest_data;

    static bool IRAM_ATTR _on_pool_ovf(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
};
