// Battery Monitor Task
// (C)2025 bekki.jp

#include "battery_monitor_task.h"

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "logger.h"
#include "util.h"

namespace receiver_system {

BatteryMonitorTask::BatteryMonitorTask()
    : Task(std::string(TASK_NAME).c_str(), PRIORITY, CORE_ID),
      mutex_(),
      battery_voltage_(0.0f),
      last_update_time_(0) {}

BatteryMonitorTask::~BatteryMonitorTask() = default;

void BatteryMonitorTask::Initialize() {
  ESP_LOGI(kTag, "BatteryMonitorTask started");

  // Configure ADC channel
  if (!adc_util::ConfigureChannel(kAdcChannel)) {
    ESP_LOGE(kTag, "Failed to configure ADC channel");
    return;
  }

  // Perform initial measurement
  MeasureBatteryVoltage();
}

void BatteryMonitorTask::Update() {
  while (true) {
    // Wait for measurement interval (30 seconds)
    util::SleepMillisecond(kMeasureIntervalMs);

    // Measure battery voltage
    MeasureBatteryVoltage();
  }
}

float BatteryMonitorTask::GetBatteryVoltage() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return battery_voltage_;
}

int64_t BatteryMonitorTask::GetLastUpdateTime() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_update_time_;
}

void BatteryMonitorTask::MeasureBatteryVoltage() {
  int adc_voltage_mv = 0;
  if (!adc_util::ReadVoltageAveraged(kAdcChannel, kAdcSampleCount, 0,
                                     &adc_voltage_mv)) {
    ESP_LOGE(kTag, "Failed to read ADC voltage");
    return;
  }

  // Calculate battery voltage
  float battery_voltage = CalculateBatteryVoltage(adc_voltage_mv);

  // Update stored values (thread-safe)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    battery_voltage_ = battery_voltage;
    last_update_time_ = esp_timer_get_time() / 1000;
  }

  ESP_LOGI(kTag, "Battery voltage: %.2f V (ADC: %d mV)", battery_voltage,
           adc_voltage_mv);
}

float BatteryMonitorTask::CalculateBatteryVoltage(int adc_voltage_mv) {
  // Apply offset correction for ESP32-C6 ADC systematic error
  int corrected_mv = adc_voltage_mv + kAdcOffsetCorrectionMv;

  // Convert ADC voltage to battery voltage using voltage divider formula
  // V_battery = V_adc / (R2 / (R1 + R2))
  float adc_voltage_v = static_cast<float>(corrected_mv) / kMvToVoltsFactor;
  float battery_voltage_v = adc_voltage_v / kVoltageDividerRatio;
  return battery_voltage_v;
}

}  // namespace receiver_system

// EOF
