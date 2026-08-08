/**
 * @file adc_dma.hpp
 * @brief Zero-queue ADC DMA driver for continuous sampling.
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_adc/adc_continuous.h>
#include <esp_err.h>

#define PIN_ADC_SENSOR_01 ADC_CHANNEL_4 // GPIO32
#define PIN_ADC_SENSOR_02 ADC_CHANNEL_5 // GPIO33
#define PIN_ADC_SENSOR_03 ADC_CHANNEL_6 // GPIO34
#define PIN_ADC_SENSOR_04 ADC_CHANNEL_7 // GPIO35
#define PIN_ADC_SENSOR_05 ADC_CHANNEL_0 // GPIO36

/** @brief Maximum internal DMA store buffer size */
constexpr uint32_t ADC_MAX_STORE_BUF_SIZE = 1024;

/**
 * @brief Enum defining logical IDs for analog sensors.
 */
enum SensorId {
    SENSOR_01 = 0,
    SENSOR_02,
    SENSOR_03,
    SENSOR_04,
    SENSOR_05,
    NUM_SENSORS
};

/**
 * @brief Configuration structure for the ADC DMA driver.
 */
struct adc_dma_config_t {
    adc_channel_t channel_map[NUM_SENSORS]; /**< Mapping of logical sensors to physical ADC channels */
    uint32_t sample_freq_hz;                /**< Sampling frequency in Hz */
    size_t dma_frame_size;                  /**< Number of bytes in each DMA frame */
};

/**
 * @brief Structure containing the latest raw ADC readings.
 */
struct adc_sensor_data_t {
    uint32_t raw[NUM_SENSORS]; /**< Array of raw ADC values */
};

/**
 * @brief Continuous ADC reading driver using DMA.
 */
class EspAdcDmaDriver {
public:
    EspAdcDmaDriver();
    ~EspAdcDmaDriver();

    /**
     * @brief Initializes the ADC DMA peripheral.
     * @param config The configuration object.
     * @return ESP_OK on success.
     */
    esp_err_t init(const adc_dma_config_t& config);

    /**
     * @brief Starts the continuous ADC reading.
     * @return ESP_OK on success.
     */
    esp_err_t start();

    /**
     * @brief Stops the continuous ADC reading.
     * @return ESP_OK on success.
     */
    esp_err_t stop();

    /**
     * @brief Polls the DMA buffer for new data. Must be called repeatedly in a task.
     * @param timeout FreeRTOS tick timeout to wait for data.
     * @return ESP_OK on success, or ESP_ERR_TIMEOUT.
     */
    esp_err_t process_dma_events(TickType_t timeout);

    /**
     * @brief Safely reads the latest snapshot of ADC data.
     * @param out_data Pointer to the structure where data will be copied.
     */
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
