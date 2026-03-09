// ESP32 B-Fox Receiver
// (C)2025 bekki.jp
// ADC Utilities - Shared ADC Unit Management

// Include ----------------------
#include "adc_util.h"

#include "logger.h"

namespace receiver_system {
namespace adc_util {

// Global ADC handles
static adc_oneshot_unit_handle_t g_adc_handle = nullptr;
static adc_cali_handle_t g_adc_cali_handle = nullptr;
static bool g_adc_calibrated = false;

bool InitializeAdcUnit() {
  if (g_adc_handle != nullptr) {
    ESP_LOGW(kTag, "ADC unit already initialized");
    return true;
  }

  // Configure ADC oneshot unit
  adc_oneshot_unit_init_cfg_t init_config = {};
  init_config.unit_id = kAdcUnit;
  init_config.clk_src = ADC_DIGI_CLK_SRC_DEFAULT;

  esp_err_t ret = adc_oneshot_new_unit(&init_config, &g_adc_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "Failed to create ADC unit: %s", esp_err_to_name(ret));
    return false;
  }

  // Initialize ADC calibration (Curve Fitting scheme for ESP32-C6)
  adc_cali_curve_fitting_config_t cali_config = {};
  cali_config.unit_id = kAdcUnit;
  cali_config.atten = kAdcAttenuation;
  cali_config.bitwidth = kAdcBitwidth;

  ret = adc_cali_create_scheme_curve_fitting(&cali_config, &g_adc_cali_handle);
  if (ret == ESP_OK) {
    g_adc_calibrated = true;
    ESP_LOGI(kTag, "ADC unit initialized with calibration (Curve Fitting)");
  } else {
    g_adc_calibrated = false;
    ESP_LOGW(kTag, "ADC unit initialized without calibration (using raw conversion)");
  }

  ESP_LOGI(kTag, "ADC unit initialized: Unit=%d, Attenuation=%d, Bitwidth=%d",
           kAdcUnit, kAdcAttenuation, kAdcBitwidth);

  return true;
}

adc_oneshot_unit_handle_t GetAdcUnitHandle() {
  return g_adc_handle;
}

adc_cali_handle_t GetAdcCaliHandle() {
  return g_adc_cali_handle;
}

bool IsAdcCalibrated() {
  return g_adc_calibrated;
}

bool ConfigureChannel(adc_channel_t channel) {
  if (g_adc_handle == nullptr) {
    ESP_LOGE(kTag, "ADC unit not initialized");
    return false;
  }

  adc_oneshot_chan_cfg_t config = {};
  config.atten = kAdcAttenuation;
  config.bitwidth = kAdcBitwidth;

  esp_err_t ret = adc_oneshot_config_channel(g_adc_handle, channel, &config);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "Failed to configure ADC channel %d: %s", channel, esp_err_to_name(ret));
    return false;
  }

  ESP_LOGI(kTag, "ADC channel %d configured", channel);
  return true;
}

bool ReadRaw(adc_channel_t channel, int* out_raw) {
  if (g_adc_handle == nullptr) {
    ESP_LOGE(kTag, "ADC unit not initialized");
    return false;
  }

  if (out_raw == nullptr) {
    ESP_LOGE(kTag, "out_raw is null");
    return false;
  }

  esp_err_t ret = adc_oneshot_read(g_adc_handle, channel, out_raw);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "ADC read failed on channel %d: %s", channel, esp_err_to_name(ret));
    return false;
  }

  return true;
}

bool ReadVoltage(adc_channel_t channel, int* out_voltage_mv) {
  if (out_voltage_mv == nullptr) {
    ESP_LOGE(kTag, "out_voltage_mv is null");
    return false;
  }

  // Read raw value
  int adc_raw = 0;
  if (!ReadRaw(channel, &adc_raw)) {
    return false;
  }

  // Convert to voltage
  if (g_adc_calibrated) {
    esp_err_t ret = adc_cali_raw_to_voltage(g_adc_cali_handle, adc_raw, out_voltage_mv);
    if (ret != ESP_OK) {
      ESP_LOGE(kTag, "ADC calibration conversion failed: %s", esp_err_to_name(ret));
      return false;
    }
  } else {
    // Fallback: simple linear conversion (not accurate)
    // 12-bit ADC: 0-4095 -> 0-3300mV
    *out_voltage_mv = (adc_raw * 3300) / 4095;
  }

  return true;
}

}  // namespace adc_util
}  // namespace receiver_system

// EOF
