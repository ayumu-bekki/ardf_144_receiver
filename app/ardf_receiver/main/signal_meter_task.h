#ifndef SIGNAL_METER_TASK_H_
#define SIGNAL_METER_TASK_H_
// (C)2025 bekki.jp
// Signal Meter Task

// Include ----------------------
#include <esp_adc/adc_oneshot.h>
#include <freertos/FreeRTOS.h>

#include <cstdint>
#include <mutex>
#include <string_view>

#include "adc_util.h"
#include "hardware_config.h"
#include "task.h"

namespace receiver_system {

class SignalMeterTask final : public Task {
 public:
  static constexpr std::string_view TASK_NAME = "SignalMeterTask";
  static constexpr int32_t PRIORITY = Task::PRIORITY_NORMAL;
  static constexpr int32_t CORE_ID = PRO_CPU_NUM;

 private:
  // Measurement interval: 100ms (10 readings/sec)
  static constexpr uint32_t kMeasureIntervalMs =
      hardware_config::kSignalMeterIntervalMs;

  // Peak hold time: 3 seconds
  static constexpr uint32_t kPeakHoldTimeMs =
      hardware_config::kSignalMeterPeakHoldMs;

  // Signal strength range (0-100)
  static constexpr int kSignalStrengthMin = 0;
  static constexpr int kSignalStrengthMax = 100;

  // ADC configuration
  static constexpr adc_channel_t kAdcChannel = ADC_CHANNEL_1;  // GPIO1

  // Sampling configuration
  static constexpr int kAdcSampleCount =
      hardware_config::kSignalMeterAdcSampleCount;
  static constexpr uint32_t kSampleIntervalMs =
      hardware_config::kSignalMeterSampleIntervalMs;

  // S-meter calibration (AGC逆特性: 信号強→電圧低)
  static constexpr int kAgcNoSignalVoltage =
      hardware_config::kSmeterAgcNoSignalVoltage;
  static constexpr int kAgcFullSignalVoltage =
      hardware_config::kSmeterAgcFullSignalVoltage;

  // S値の最大値 (0-9)
  static constexpr int kAgcSValueMax = hardware_config::kSmeterSValueMax;

  // S-meter パーセント基準値 (0-100%)
  static constexpr int kSmeterPercentMax = hardware_config::kSmeterPercentMax;

  // EMAフィルタ平滑化係数
  static constexpr float kEmaAlpha = hardware_config::kSmeterEmaAlpha;

 public:
  SignalMeterTask();
  ~SignalMeterTask();

  void Initialize() override;
  void Update() override;

  // Thread-safe signal strength reading
  // Returns: 0-100 (0% to 100% signal strength)
  int GetSignalStrength() const;

  // Returns: 0-100 (peak signal strength within last 3 seconds)
  int GetPeakSignalStrength() const;

  // Returns: 0-100 (S-meter linear value for bar display)
  int GetSmeterRaw() const;

  // Returns: 0-100 (peak S-meter linear value within last 3 seconds)
  int GetPeakSmeterRaw() const;

  // Returns: ADC raw voltage in mV
  int GetAdcRawValue() const;

  int64_t GetLastUpdateTime() const;

  // S-meter calculation based on AGC inverse characteristic:
  // >= 1075mV (no signal) -> 0%
  // <= 920mV (full signal) -> 100%
  // Linear interpolation, then logarithmic for display

 private:
  void MeasureSignalStrength();
  int CalculateSignalStrength(int adc_voltage_mv);
  int CalculateSmeterRaw(
      int adc_voltage_mv);  // Calculate 0-100 value for S-meter
  void UpdatePeakValue(int signal_strength, int smeter_raw);

 private:
  mutable std::mutex mutex_;
  int signal_strength_;        // Current signal strength (0-100, logarithmic)
  int peak_signal_strength_;   // Peak signal strength (0-100, logarithmic)
  int smeter_raw_;             // S-meter raw value (0-100, linear)
  int peak_smeter_raw_;        // Peak S-meter raw value (0-100, linear)
  int adc_raw_value_;         // ADC filtered voltage in mV (EMA applied)
  float ema_voltage_mv_;      // EMA filter state (-1 = uninitialized)
  int64_t last_update_time_;  // Last update time (ms)
  int64_t peak_update_time_;  // Peak update time (ms)
};

/*
  OLEDディスプレイで表示する場合:
  // 現在値とピーク値を取得
  int current = signal_meter_task_.GetSignalStrength();
  int peak = signal_meter_task_.GetPeakSignalStrength();

  // S0-S9に変換
  int s_current = current / 11;
  int s_peak = peak / 11;
  if (s_current > 9) s_current = 9;
  if (s_peak > 9) s_peak = 9;

  // 表示
  sprintf(buf, "S%d (Peak:S%d)", s_current, s_peak);
*/

}  // namespace receiver_system

#endif  // SIGNAL_METER_TASK_H_

// EOF
