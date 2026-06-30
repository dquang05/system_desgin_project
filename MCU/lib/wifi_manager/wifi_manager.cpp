#include "wifi_manager.hpp"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include <cstring>

#define CHECK_RET(x) do { esp_err_t _err = (x); if (_err != ESP_OK) return _err; } while(0)

namespace wifi_manager {

static const char *TAG = "WIFI_MANAGER";
static bool sys_initialized = false;

WifiManager::WifiManager()
    : _retry_count(0), _netif(nullptr), _initialized(false),
      _is_intentional_stop(false), _udp_sock(-1) {
  _wifi_event_group = xEventGroupCreateStatic(&_event_group_buffer);
}

WifiManager::~WifiManager() {
  deinit();
}

void WifiManager::wifi_event_handler(void *arg, esp_event_base_t event_base,
                                     int32_t event_id, void *event_data) {
  WifiManager *instance = static_cast<WifiManager *>(arg);

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    instance->_is_intentional_stop = false;
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (instance->_is_intentional_stop) {
      xEventGroupClearBits(instance->_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
      ESP_LOGI(TAG, "Wi-Fi disconnected intentionally.");
    } else if (instance->_retry_count < instance->_config.max_retry) {
      esp_wifi_connect();
      instance->_retry_count++;
      ESP_LOGI(TAG, "Retrying to connect to the AP...");
    } else {
      xEventGroupSetBits(instance->_wifi_event_group, WIFI_FAIL_BIT);
      ESP_LOGE(TAG, "Failed to connect to the AP.");
    }
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event =
        reinterpret_cast<ip_event_got_ip_t *>(event_data);
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    instance->_retry_count = 0;
    xEventGroupSetBits(instance->_wifi_event_group, WIFI_CONNECTED_BIT);
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t *event =
        reinterpret_cast<wifi_event_ap_staconnected_t *>(event_data);
    ESP_LOGI(TAG, "Station " MACSTR " joined, AID=%d", MAC2STR(event->mac),
             event->aid);
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    wifi_event_ap_stadisconnected_t *event =
        reinterpret_cast<wifi_event_ap_stadisconnected_t *>(event_data);
    ESP_LOGI(TAG, "Station " MACSTR " left, AID=%d", MAC2STR(event->mac),
             event->aid);
  }
}

esp_err_t WifiManager::init(const WifiConfig &config) {
  if (_initialized) {
    ESP_LOGW(TAG, "Wi-Fi already initialized.");
    return ESP_OK;
  }

  _config = config;

  if (!sys_initialized) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      CHECK_RET(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    CHECK_RET(ret);

    CHECK_RET(esp_netif_init());
    CHECK_RET(esp_event_loop_create_default());
    sys_initialized = true;
  }

  if (_config.mode == WifiMode::MODE_STA) {
    _netif = esp_netif_create_default_wifi_sta();
  } else {
    _netif = esp_netif_create_default_wifi_ap();
  }

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  CHECK_RET(esp_wifi_init(&cfg));

  CHECK_RET(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, this,
      &_instance_any_id));
  CHECK_RET(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, this,
      &_instance_got_ip));

  wifi_config_t wifi_cfg = {};
  if (_config.mode == WifiMode::MODE_STA) {
    std::strncpy(reinterpret_cast<char*>(wifi_cfg.sta.ssid), _config.ssid, sizeof(wifi_cfg.sta.ssid));
    std::strncpy(reinterpret_cast<char*>(wifi_cfg.sta.password), _config.password, sizeof(wifi_cfg.sta.password));
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    CHECK_RET(esp_wifi_set_mode(WIFI_MODE_STA));
    CHECK_RET(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
  } else {
    std::strncpy(reinterpret_cast<char*>(wifi_cfg.ap.ssid), _config.ssid, sizeof(wifi_cfg.ap.ssid));
    std::strncpy(reinterpret_cast<char*>(wifi_cfg.ap.password), _config.password, sizeof(wifi_cfg.ap.password));
    wifi_cfg.ap.ssid_len = std::strlen(_config.ssid);
    wifi_cfg.ap.max_connection = 4;
    wifi_cfg.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    if (std::strlen(_config.password) == 0) {
      wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;
    }

    CHECK_RET(esp_wifi_set_mode(WIFI_MODE_AP));
    CHECK_RET(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
  }

  CHECK_RET(esp_wifi_start());
  _initialized = true;
  ESP_LOGI(TAG, "Wi-Fi initialization finished.");
  return ESP_OK;
}

esp_err_t WifiManager::deinit() {
  if (!_initialized) return ESP_OK;

  esp_wifi_stop();
  
  esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, _instance_got_ip);
  esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, _instance_any_id);
  
  esp_wifi_deinit();
  
  if (_netif) {
    esp_netif_destroy_default_wifi(_netif);
    _netif = nullptr;
  }

  if (_udp_sock >= 0) {
    close(_udp_sock);
    _udp_sock = -1;
  }

  _initialized = false;
  ESP_LOGI(TAG, "Wi-Fi deinitialized.");
  return ESP_OK;
}

void WifiManager::start() {
  if (_initialized) {
    _is_intentional_stop = false;
    _retry_count = 0;
    xEventGroupClearBits(_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    esp_wifi_start();
    ESP_LOGI(TAG, "Wi-Fi started.");
  }
}

void WifiManager::stop() {
  if (_initialized) {
    _is_intentional_stop = true;
    esp_wifi_stop();
    
    if (_udp_sock >= 0) {
      close(_udp_sock);
      _udp_sock = -1;
    }
    
    ESP_LOGI(TAG, "Wi-Fi stopped.");
  }
}

bool WifiManager::wait_for_connection(uint32_t timeout_ms) {
  EventBits_t bits =
      xEventGroupWaitBits(_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                          pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout_ms));

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "Connected to AP.");
    return true;
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to AP.");
    return false;
  }

  ESP_LOGE(TAG, "Timeout waiting for Wi-Fi connection.");
  return false;
}

bool WifiManager::is_connected() const {
  if (!_initialized) return false;
  EventBits_t bits = xEventGroupGetBits(_wifi_event_group);
  return (bits & WIFI_CONNECTED_BIT) != 0;
}

bool WifiManager::send_log_data(const char *ip, uint16_t port,
                                const uint8_t *data, size_t len) {
  if (ip == nullptr || data == nullptr || len == 0) {
    return false;
  }

  if (_udp_sock < 0) {
    _udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (_udp_sock < 0) {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
      return false;
    }
  }

  struct sockaddr_in dest_addr = {};
  dest_addr.sin_addr.s_addr = inet_addr(ip);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(port);

  int err = sendto(_udp_sock, data, len, 0,
                   reinterpret_cast<struct sockaddr *>(&dest_addr),
                   sizeof(dest_addr));
  if (err < 0) {
    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    // Nếu lỗi gửi, có thể do mạng, ta tạm đóng socket để lần sau tạo lại
    close(_udp_sock);
    _udp_sock = -1;
    return false;
  }

  return true;
}

} // namespace wifi_manager
