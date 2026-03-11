#ifndef BFOX_RECEIVER_MAIN_ADC_UTIL_H_
#define BFOX_RECEIVER_MAIN_ADC_UTIL_H_
// ESP32 B-Fox Receiver
// (C)2025 bekki.jp
// ADC Utilities - Shared ADC Unit Management

// Include ----------------------
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>

namespace receiver_system {
namespace adc_util {

// ADC configuration constants
constexpr adc_unit_t kAdcUnit = ADC_UNIT_1;
constexpr adc_atten_t kAdcAttenuation = ADC_ATTEN_DB_12;  // 0-3.3V
constexpr adc_bitwidth_t kAdcBitwidth = ADC_BITWIDTH_12;  // 12-bit
constexpr int kAdcTimeoutMs = 1000;                       // 1 second timeout

// フォールバック線形変換係数 (キャリブレーション未使用時)
// 12-bit ADC: raw 0-4095 → 0-3300mV
constexpr int kAdcFallbackMaxMv = 3300;
constexpr int kAdcFallbackMaxRaw = 4095;

// Initialize ADC unit (call once at startup)
// Returns true on success
bool InitializeAdcUnit();

// Get ADC unit handle (shared by all channels)
adc_oneshot_unit_handle_t GetAdcUnitHandle();

// Get ADC calibration handle (shared by all channels)
adc_cali_handle_t GetAdcCaliHandle();

// Check if ADC is calibrated
bool IsAdcCalibrated();

// Configure a specific ADC channel
// Must be called after InitializeAdcUnit()
bool ConfigureChannel(adc_channel_t channel);

// Read ADC raw value from a channel
// Returns true on success, raw value in *out_raw
bool ReadRaw(adc_channel_t channel, int* out_raw);

// Read ADC voltage from a channel (using calibration if available)
// Returns true on success, voltage in mV in *out_voltage_mv
bool ReadVoltage(adc_channel_t channel, int* out_voltage_mv);

// 複数サンプリング平均（サンプル間に sample_interval_ms のスリープを挟む）。
// 1サンプルでも成功すれば true を返す。sample_interval_ms = 0 の場合はスリープなし。
bool ReadVoltageAveraged(adc_channel_t channel, int sample_count,
                         uint32_t sample_interval_ms, int* out_voltage_mv);

}  // namespace adc_util
}  // namespace receiver_system

#endif  // BFOX_RECEIVER_MAIN_ADC_UTIL_H_

// EOF
