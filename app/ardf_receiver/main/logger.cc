// ESP32 B-Fox Receiver
// (C)2025 bekki.jp

// Include ----------------------
#include "logger.h"

namespace receiver_system {
namespace logger {

void InitializeLogLevel() {
  esp_log_level_set("*", ESP_LOG_WARN);
  esp_log_level_set(kTag, CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_VERBOSE
                              ? ESP_LOG_VERBOSE
                              : ESP_LOG_INFO);
}

}  // namespace logger
}  // namespace receiver_system
