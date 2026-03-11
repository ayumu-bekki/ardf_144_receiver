#ifndef BATTERY_MONITOR_TASK_H_
#define BATTERY_MONITOR_TASK_H_
// (C)2025 bekki.jp
// Battery Monitor Task

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

class BatteryMonitorTask final : public Task {
 public:
  static constexpr std::string_view TASK_NAME = "BatteryMonitorTask";
  static constexpr int32_t PRIORITY = Task::PRIORITY_LOW;
  static constexpr int32_t CORE_ID = PRO_CPU_NUM;

 private:
  // Measurement interval: 30 seconds
  static constexpr uint32_t kMeasureIntervalMs =
      hardware_config::kBatteryMonitorIntervalMs;

  // Voltage divider circuit constants
  // R1 (battery side): 150kΩ, R2 (GND side): 47kΩ
  static constexpr float kVoltageDividerR1 =
      hardware_config::kBatteryVoltageDividerR1;
  static constexpr float kVoltageDividerR2 =
      hardware_config::kBatteryVoltageDividerR2;
  static constexpr float kVoltageDividerRatio =
      kVoltageDividerR2 / (kVoltageDividerR1 + kVoltageDividerR2);

  // Number of ADC samples to average
  static constexpr int kAdcSampleCount =
      hardware_config::kBatteryAdcSampleCount;

  // ADC offset correction (mV)
  static constexpr int kAdcOffsetCorrectionMv =
      hardware_config::kBatteryAdcOffsetCorrectionMv;

  // mV → V 変換係数
  static constexpr float kMvToVoltsFactor = 1000.0f;

  // ADC configuration
  static constexpr adc_channel_t kAdcChannel = ADC_CHANNEL_0;  // GPIO0

 public:
  BatteryMonitorTask();
  ~BatteryMonitorTask();

  void Initialize() override;
  void Update() override;

  // Thread-safe battery voltage reading
  float GetBatteryVoltage() const;
  int64_t GetLastUpdateTime() const;

 private:
  void MeasureBatteryVoltage();
  float CalculateBatteryVoltage(int adc_voltage_mv);

 private:
  mutable std::mutex mutex_;
  float battery_voltage_;     // Latest battery voltage (V)
  int64_t last_update_time_;  // Last update time (ms)
};

}  // namespace receiver_system

#endif  // BATTERY_MONITOR_TASK_H_

// EOF
