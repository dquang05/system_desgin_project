/**
 * @file    main.c
 * @brief   Blink LED cơ bản trên ESP32 WROOM-32
 *          LED tích hợp nằm trên GPIO2
 *
 * @board   ESP32-WROOM-32
 * @sdk     ESP-IDF
 */

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

/*  Định nghĩa chân LED
 */
#define BLINK_GPIO GPIO_NUM_2 // GPIO2 = LED tích hợp trên ESP-WROOM-32
#define BLINK_PERIOD_MS 3000  // Chu kỳ nhấp nháy (ms)

/* ─── Hàm khởi tạo GPIO ──────────────────────────────────────────────────── */
static void led_init(void) {
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << BLINK_GPIO), // Chọn GPIO2
      .mode = GPIO_MODE_OUTPUT,             // Chế độ output
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE, // Không dùng ngắt
  };
  gpio_config(&io_conf);
}

/* ─── app_main ───────────────────────────────────────────────────────────── */
void app_main_backup(void) {
  led_init();

  uint8_t led_state = 0;

  while (1) {
    gpio_set_level(BLINK_GPIO, led_state);

    led_state ^= 1; // Toggle trạng thái

    vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD_MS));
  }
}