// Receiver Settings (NVS Storage)
// (C)2025 bekki.jp

#include "receiver_settings.h"

#include "logger.h"

namespace receiver_system {

ReceiverSettings::ReceiverSettings()
    : nvs_handle_(0),
      nvs_handle_open_(false),
      frequency_hz_(kDefaultFrequency),
      preamp_enabled_(kDefaultPreamp),
      volume_(kDefaultVolume) {}

ReceiverSettings::~ReceiverSettings() { CloseNvsHandle(); }

bool ReceiverSettings::OpenNvsHandle() {
  if (nvs_handle_open_) {
    return true;
  }
  esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &nvs_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "Failed to open NVS: %s", esp_err_to_name(err));
    return false;
  }
  nvs_handle_open_ = true;
  return true;
}

void ReceiverSettings::CloseNvsHandle() {
  if (nvs_handle_open_) {
    nvs_close(nvs_handle_);
    nvs_handle_open_ = false;
  }
}

bool ReceiverSettings::Load() {
  nvs_handle_t read_handle;
  esp_err_t err = nvs_open(kNvsNamespace, NVS_READONLY, &read_handle);
  if (err != ESP_OK) {
    ESP_LOGI(kTag, "No saved settings, using defaults");
    frequency_hz_ = kDefaultFrequency;
    preamp_enabled_ = kDefaultPreamp;
    volume_ = kDefaultVolume;
    return false;
  }

  // Initialize with default values
  uint32_t freq = kDefaultFrequency;
  uint8_t preamp = kDefaultPreamp ? 1 : 0;
  uint8_t vol = kDefaultVolume;

  // Load saved values (keep defaults if not found)
  nvs_get_u32(read_handle, kKeyFrequency, &freq);
  nvs_get_u8(read_handle, kKeyPreamp, &preamp);
  nvs_get_u8(read_handle, kKeyVolume, &vol);

  nvs_close(read_handle);

  frequency_hz_ = freq;
  preamp_enabled_ = (preamp != 0);
  volume_ = vol;

  ESP_LOGI(kTag, "Settings loaded: freq=%u Hz, preamp=%d, vol=%u",
           frequency_hz_, preamp_enabled_, volume_);

  // Open READWRITE handle to keep for subsequent saves
  OpenNvsHandle();

  return true;
}

bool ReceiverSettings::SaveFrequency(uint32_t freq_hz) {
  frequency_hz_ = freq_hz;

  if (!OpenNvsHandle()) return false;

  esp_err_t err = nvs_set_u32(nvs_handle_, kKeyFrequency, frequency_hz_);
  if (err == ESP_OK) {
    err = nvs_commit(nvs_handle_);
  }

  if (err != ESP_OK) {
    ESP_LOGE(kTag, "Failed to save frequency: %s", esp_err_to_name(err));
    return false;
  }

  ESP_LOGD(kTag, "Frequency saved: %u Hz", frequency_hz_);
  return true;
}

bool ReceiverSettings::SavePreamp(bool enabled) {
  preamp_enabled_ = enabled;

  if (!OpenNvsHandle()) return false;

  esp_err_t err = nvs_set_u8(nvs_handle_, kKeyPreamp, enabled ? 1 : 0);
  if (err == ESP_OK) {
    err = nvs_commit(nvs_handle_);
  }

  if (err != ESP_OK) {
    ESP_LOGE(kTag, "Failed to save preamp: %s", esp_err_to_name(err));
    return false;
  }

  ESP_LOGD(kTag, "Preamp saved: %s", enabled ? "Enable" : "Disable");
  return true;
}

bool ReceiverSettings::SaveVolume(uint8_t volume) {
  volume_ = volume;

  if (!OpenNvsHandle()) return false;

  esp_err_t err = nvs_set_u8(nvs_handle_, kKeyVolume, volume_);
  if (err == ESP_OK) {
    err = nvs_commit(nvs_handle_);
  }

  if (err != ESP_OK) {
    ESP_LOGE(kTag, "Failed to save volume: %s", esp_err_to_name(err));
    return false;
  }

  ESP_LOGD(kTag, "Volume saved: %u", volume_);
  return true;
}

}  // namespace receiver_system
