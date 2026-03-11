#ifndef RECEIVER_MAIN_RECEIVER_SETTINGS_H_
#define RECEIVER_MAIN_RECEIVER_SETTINGS_H_
// Receiver Settings (NVS Storage)
// (C)2025 bekki.jp

// Include ----------------------
#include <nvs.h>

#include <cstdint>
#include <memory>

namespace receiver_system {

/// Receiver Settings Manager (NVS-based persistence)
class ReceiverSettings final {
 public:
  ReceiverSettings();
  ~ReceiverSettings();

  // Load settings from NVS (call at startup)
  bool Load();

  // Save individual settings immediately
  bool SaveFrequency(uint32_t freq_hz);
  bool SavePreamp(bool enabled);
  bool SaveVolume(uint8_t volume);

  // Getters
  uint32_t GetFrequencyHz() const { return frequency_hz_; }
  bool GetPreampEnabled() const { return preamp_enabled_; }
  uint8_t GetVolume() const { return volume_; }

 private:
  bool OpenNvsHandle();
  void CloseNvsHandle();

  nvs_handle_t nvs_handle_;
  bool nvs_handle_open_;
  uint32_t frequency_hz_;
  bool preamp_enabled_;
  uint8_t volume_;

  // NVS keys
  static constexpr const char* kNvsNamespace = "receiver";
  static constexpr const char* kKeyFrequency = "frequency";
  static constexpr const char* kKeyPreamp = "preamp";
  static constexpr const char* kKeyVolume = "volume";

  // Default values
  static constexpr uint32_t kDefaultFrequency = 145000000;  // 145.00 MHz
  static constexpr bool kDefaultPreamp = true;
  static constexpr uint8_t kDefaultVolume = 15;
};

using ReceiverSettingsSharedPtr = std::shared_ptr<ReceiverSettings>;
using ReceiverSettingsWeakPtr = std::weak_ptr<ReceiverSettings>;

}  // namespace receiver_system

#endif  // RECEIVER_MAIN_RECEIVER_SETTINGS_H_
