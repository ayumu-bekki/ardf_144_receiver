#ifndef BFOX_RECEIVER_MAIN_LOGGER_H_
#define BFOX_RECEIVER_MAIN_LOGGER_H_
// ESP32 B-Fox Receiver
// (C)2025 bekki.jp

// Include ----------------------
#include <esp_log.h>

// systen Loglevel Redefine
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

namespace receiver_system {

/// Application Log Tag
static constexpr char kTag[] = "ARDFReceiver";

namespace logger {

void InitializeLogLevel();

}  // namespace logger
}  // namespace receiver_system

#endif  // BFOX_RECEIVER_MAIN_LOGGER_H_
