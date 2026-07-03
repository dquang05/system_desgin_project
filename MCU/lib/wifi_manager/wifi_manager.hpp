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

/** @brief Default Wi-Fi SSID */
constexpr char DEFAULT_WIFI_SSID[] = "Danh Van";
/** @brief Default Wi-Fi Password */
constexpr char DEFAULT_WIFI_PASSWORD[] = "danhvan123";
/** @brief Default connection retry limit */
constexpr uint8_t DEFAULT_MAX_RETRY = 3;

/** @brief Wi-Fi Operating Modes */
enum class WifiMode { MODE_STA, MODE_AP };

/** @brief Default Wi-Fi Mode */
constexpr WifiMode DEFAULT_WIFI_MODE = WifiMode::MODE_STA;

/**
 * @brief Configuration structure for the Wi-Fi Manager.
 */
struct WifiConfig {
  WifiMode mode;
  char ssid[32];
  char password[64];
  uint8_t max_retry;

  /**
   * @brief Construct a new Wifi Config object.
   * 
   * @param m Wi-Fi Mode (STA or AP).
   * @param s SSID string.
   * @param p Password string.
   * @param retry Maximum connection retries.
   */
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

/**
 * @brief Thread-safe Wi-Fi Manager class.
 */
class WifiManager {
public:
  WifiManager();
  ~WifiManager();

  /**
   * @brief Initializes the Wi-Fi driver and establishes connection or starts AP.
   * 
   * @param config The configuration object.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t init(const WifiConfig &config = WifiConfig());

  /**
   * @brief Deinitializes the Wi-Fi stack and frees resources.
   * 
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t deinit();

  /**
   * @brief Starts the Wi-Fi interface if previously stopped.
   */
  void start();

  /**
   * @brief Stops the Wi-Fi interface intentionally.
   */
  void stop();

  /**
   * @brief Blocks until STA connection is established.
   * 
   * @param timeout_ms Timeout in milliseconds.
   * @return true if connected.
   * @return false if timeout occurred.
   */
  bool wait_for_connection(uint32_t timeout_ms);

  /**
   * @brief Checks if the Wi-Fi is currently connected (non-blocking).
   * 
   * @return true if connected.
   * @return false otherwise.
   */
  bool is_connected() const;

  /**
   * @brief Sends telemetry data via UDP.
   * 
   * @param ip Target IP address.
   * @param port Target UDP port.
   * @param data Pointer to the payload.
   * @param len Length of the payload.
   * @return true on success.
   * @return false on failure.
   */
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
