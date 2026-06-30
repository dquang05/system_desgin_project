#pragma once

#include "esp_event.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <array>
#include <cstdint>
#include <cstring>
#include <atomic>

namespace wifi_manager {

// Default Configuration Constants
constexpr char DEFAULT_WIFI_SSID[] = "Danh Van";
constexpr char DEFAULT_WIFI_PASSWORD[] = "danhvan123";
constexpr uint8_t DEFAULT_MAX_RETRY = 3;

enum class WifiMode { MODE_STA, MODE_AP };

constexpr WifiMode DEFAULT_WIFI_MODE = WifiMode::MODE_STA;

struct WifiConfig {
  WifiMode mode;
  char ssid[32];
  char password[64];
  uint8_t max_retry;

  WifiConfig(WifiMode m = DEFAULT_WIFI_MODE, const char *s = DEFAULT_WIFI_SSID,
             const char *p = DEFAULT_WIFI_PASSWORD,
             uint8_t retry = DEFAULT_MAX_RETRY) {
    mode = m;
    std::strncpy(ssid, s, sizeof(ssid) - 1);
    ssid[sizeof(ssid) - 1] = '\0';
    std::strncpy(password, p, sizeof(password) - 1);
    password[sizeof(password) - 1] = '\0';
    max_retry = retry;
  }
};

class WifiManager {
public:
  WifiManager();
  ~WifiManager();

  // Initialize Wi-Fi driver and connect/start AP
  esp_err_t init(const WifiConfig &config = WifiConfig());

  // Deinitialize Wi-Fi to free resources
  esp_err_t deinit();

  // Start Wi-Fi (if previously stopped)
  void start();

  // Stop Wi-Fi
  void stop();

  // Wait for STA connection (only applicable for MODE_STA)
  bool wait_for_connection(uint32_t timeout_ms);

  // Check if Wi-Fi is currently connected (non-blocking, silent)
  bool is_connected() const;

  // Send a dummy log via UDP
  bool send_log_data(const char *ip, uint16_t port, const uint8_t *data,
                     size_t len);

private:
  static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data);

  WifiConfig _config;
  EventGroupHandle_t _wifi_event_group;
  StaticEventGroup_t _event_group_buffer;
  uint8_t _retry_count;
  esp_netif_t *_netif;

  esp_event_handler_instance_t _instance_any_id;
  esp_event_handler_instance_t _instance_got_ip;

  bool _initialized;
  std::atomic<bool> _is_intentional_stop;
  int _udp_sock;

  static constexpr uint32_t WIFI_CONNECTED_BIT = BIT0;
  static constexpr uint32_t WIFI_FAIL_BIT = BIT1;
};

} // namespace wifi_manager
