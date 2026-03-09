// Signal Meter Task
// (C)2025 bekki.jp

#include "signal_meter_task.h"

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "logger.h"
#include "util.h"

namespace receiver_system {

SignalMeterTask::SignalMeterTask()
    : Task(std::string(TASK_NAME).c_str(), PRIORITY, CORE_ID),
      mutex_(),
      signal_strength_(0),
      peak_signal_strength_(0),
      smeter_raw_(0),
      peak_smeter_raw_(0),
      adc_raw_value_(0),
      ema_voltage_mv_(-1),
      last_update_time_(0),
      peak_update_time_(0) {}

SignalMeterTask::~SignalMeterTask() = default;

void SignalMeterTask::Initialize() {
  ESP_LOGI(kTag, "SignalMeterTask started");

  // Configure ADC channel
  ESP_LOGI(kTag, "Configuring ADC channel %d for GPIO1 (A1)", kAdcChannel);
  if (!adc_util::ConfigureChannel(kAdcChannel)) {
    ESP_LOGE(kTag, "Failed to configure ADC channel %d", kAdcChannel);
    return;
  }

  ESP_LOGI(kTag, "ADC channel %d configured successfully", kAdcChannel);

  // Skip initial measurement to avoid interfering with I2C device initialization
  // Measurement will start in Update() loop
}

void SignalMeterTask::Update() {
  while (true) {
    // Wait for measurement interval (100ms = 10 readings/sec)
    util::SleepMillisecond(kMeasureIntervalMs);

    // Measure signal strength
    MeasureSignalStrength();
  }
}

int SignalMeterTask::GetSignalStrength() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return signal_strength_;
}

int SignalMeterTask::GetPeakSignalStrength() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return peak_signal_strength_;
}

int SignalMeterTask::GetSmeterRaw() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return smeter_raw_;
}

int SignalMeterTask::GetPeakSmeterRaw() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return peak_smeter_raw_;
}

int SignalMeterTask::GetAdcRawValue() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return adc_raw_value_;
}

uint32_t SignalMeterTask::GetLastUpdateTime() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_update_time_;
}

void SignalMeterTask::MeasureSignalStrength() {
  // Take multiple samples and average them to reduce noise
  int adc_voltage_sum = 0;
  int valid_samples = 0;

  for (int i = 0; i < kAdcSampleCount; ++i) {
    int adc_voltage_mv = 0;

    if (adc_util::ReadVoltage(kAdcChannel, &adc_voltage_mv)) {
      adc_voltage_sum += adc_voltage_mv;
      valid_samples++;
    }

    // Small delay between samples (1ms)
    util::SleepMillisecond(1);
  }

  if (valid_samples == 0) {
    ESP_LOGE(kTag, "Failed to read any ADC samples from channel %d", kAdcChannel);
    return;
  }

  // Calculate average
  int adc_voltage_mv = adc_voltage_sum / valid_samples;

  // EMAフィルタ適用
  if (ema_voltage_mv_ < 0) {
    ema_voltage_mv_ = static_cast<float>(adc_voltage_mv);  // 初回は即値で初期化
  } else {
    ema_voltage_mv_ = kEmaAlpha * static_cast<float>(adc_voltage_mv)
                    + (1.0f - kEmaAlpha) * ema_voltage_mv_;
  }
  int filtered_voltage_mv = static_cast<int>(ema_voltage_mv_);

  // Calculate signal strength (0-100, logarithmic)
  int signal_strength = CalculateSignalStrength(filtered_voltage_mv);

  // Calculate S-meter raw value (0-100, linear for S-meter)
  int smeter_raw = CalculateSmeterRaw(filtered_voltage_mv);

  // Update stored values (thread-safe)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    signal_strength_ = signal_strength;
    smeter_raw_ = smeter_raw;
    adc_raw_value_ = filtered_voltage_mv;
    last_update_time_ = esp_timer_get_time() / 1000;

    // Update peak value
    UpdatePeakValue(signal_strength, smeter_raw);
  }

  // Calculate S-meter value (0-9) based on AGC inverse characteristic
  // >= 1126mV (no signal) -> S0
  // <= 800mV (full signal) -> S9
  int s_meter = 0;
  if (filtered_voltage_mv >= kAgcNoSignalVoltage) {
    s_meter = 0;
  } else if (filtered_voltage_mv <= kAgcFullSignalVoltage) {
    s_meter = 9;
  } else {
    // Linear interpolation: 1126mV->S0, 800mV->S9
    float voltage_range = kAgcNoSignalVoltage - kAgcFullSignalVoltage;  // 326mV
    float voltage_offset = kAgcNoSignalVoltage - filtered_voltage_mv;
    s_meter = static_cast<int>((voltage_offset / voltage_range) * 9.0f);
    if (s_meter < 0) s_meter = 0;
    if (s_meter > 9) s_meter = 9;
  }

  // Calculate peak S-meter
  int peak_s_meter = (peak_smeter_raw_ * 9) / 100;
  if (peak_s_meter > 9) {
    peak_s_meter = 9;
  }

  ESP_LOGI(kTag, "ADC: %4dmV ema:%4dmV | raw:%3d peak:%3d | S%d (peak S%d)",
           adc_voltage_mv, filtered_voltage_mv, smeter_raw, peak_smeter_raw_,
           s_meter, peak_s_meter);
}

int SignalMeterTask::CalculateSmeterRaw(int adc_voltage_mv) {
  // AGC逆特性: 高電圧=弱信号、低電圧=強信号
  if (adc_voltage_mv >= kAgcNoSignalVoltage) {
    return 0;
  }
  if (adc_voltage_mv <= kAgcFullSignalVoltage) {
    return 100;
  }

  // 線形補間: 1126mV→0%, 800mV→100%
  float voltage_range = kAgcNoSignalVoltage - kAgcFullSignalVoltage;  // 326mV
  float voltage_offset = kAgcNoSignalVoltage - adc_voltage_mv;
  int smeter_raw = static_cast<int>((voltage_offset / voltage_range) * 100.0f);
  if (smeter_raw < 0) {
    smeter_raw = 0;
  }
  if (smeter_raw > 100) { 
    smeter_raw = 100;
  }

  return smeter_raw;
}

int SignalMeterTask::CalculateSignalStrength(int adc_voltage_mv) {
  int linear = CalculateSmeterRaw(adc_voltage_mv);  // 0-100
  if (linear == 0) {
    return 0;
  }
  // 対数変換: linear=1→～17, linear=100→100
  float log_val = log10f(static_cast<float>(linear)) / log10f(100.0f) * 100.0f;
  int result = static_cast<int>(log_val);
  if (result < 0) { 
    result = 0;
  } else if (result > 100) {
    result = 100;
  }
  return result;
}

void SignalMeterTask::UpdatePeakValue(int signal_strength, int smeter_raw) {
  // This method is called from within a locked context, so no mutex needed
  uint32_t current_time = esp_timer_get_time() / 1000;

  // Update peak if current value is higher
  if (signal_strength > peak_signal_strength_) {
    peak_signal_strength_ = signal_strength;
    peak_update_time_ = current_time;
  }

  if (smeter_raw > peak_smeter_raw_) {
    peak_smeter_raw_ = smeter_raw;
  }

  // Reset peak value after hold time (3 seconds)
  if (current_time - peak_update_time_ >= kPeakHoldTimeMs) {
    peak_signal_strength_ = signal_strength_;
    peak_smeter_raw_ = smeter_raw_;
    peak_update_time_ = current_time;
  }
}

}  // namespace receiver_system

// EOF
