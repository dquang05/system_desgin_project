#include "../lib/adc_dma/adc_dma.hpp"
#include "freertos/idf_additions.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "AdcTest";
static EspAdcDmaDriver adc_driver;

void adc_worker_task(void *arg) {
  ESP_LOGI(TAG, "ADC worker task started on core %d", xPortGetCoreID());
  while (true) {
    adc_driver.process_dma_events(pdMS_TO_TICKS(100));
  }
}

void adc_test_task(void *arg) {
  adc_sensor_data_t data;

  while (true) {
    adc_driver.read_sensor_data(&data);

    ESP_LOGI(TAG,
             "Sensor 1: %d mV, Sensor 2: %d mV, Sensor 3: %d mV, Sensor 4: %d "
             "mV, Sensor 5: %d mV",
             data.get_voltage(SENSOR_01), data.get_voltage(SENSOR_02),
             data.get_voltage(SENSOR_03), data.get_voltage(SENSOR_04),
             data.get_voltage(SENSOR_05));

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

extern "C" void app_main_adc() {
  adc_dma_config_t config = {};
  config.channel_map[SENSOR_01] = PIN_ADC_SENSOR_01;
  config.channel_map[SENSOR_02] = PIN_ADC_SENSOR_02;
  config.channel_map[SENSOR_03] = PIN_ADC_SENSOR_03;
  config.channel_map[SENSOR_04] = PIN_ADC_SENSOR_04;
  config.channel_map[SENSOR_05] = PIN_ADC_SENSOR_05;
  config.sample_freq_hz = 20000;
  config.dma_frame_size = 256;

  esp_err_t err = adc_driver.init(config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ADC Init failed");
    return;
  }
  ESP_LOGI(TAG, "ADC Initialized");

  err = adc_driver.start();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ADC Start failed");
    return;
  }
  ESP_LOGI(TAG, "ADC Started");

  xTaskCreatePinnedToCore(adc_worker_task, "adc_worker", 4096, nullptr, 5,
                          nullptr, 1);
  xTaskCreate(adc_test_task, "adc_test", 4096, nullptr, 4, nullptr);
}
