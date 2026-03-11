#ifndef RECEIVER_MAIN_RECEIVER_SYSTEM_H_
#define RECEIVER_MAIN_RECEIVER_SYSTEM_H_
// ReceiverSystem
// (C)2025 bekki.jp

// Include ----------------------
#include <memory>

#include "battery_monitor_task.h"
#include "gpio_input_watch_task.h"
#include "mcp4018.h"
#include "receiver_settings.h"
#include "receiver_system_interface.h"
#include "si5351a.h"
#include "signal_meter_task.h"
#include "ssd1306.h"
#include "ui_controller_task.h"

namespace receiver_system {

/// ReceiverSystem
class ReceiverSystem final
    : public ReceiverSystemInterface,
      public std::enable_shared_from_this<ReceiverSystem> {
 public:
  ReceiverSystem();
  ~ReceiverSystem();

  void Start();

  // ReceiverSystemInterface implementation
  void SetFrequency(uint32_t freq_hz) override;
  void SetPreampEnable(bool enable) override;
  void SetVolume(uint8_t volume) override;
  int GetSignalStrength() const override;
  int GetPeakSignalStrength() const override;
  float GetBatteryVoltage() const override;

 private:
  Si5351A si5351a_;
  MCP4018 mcp4018_;
  SSD1306SharedPtr ssd1306_;
  GpioInputWatchTask gpio_watch_task_;
  UiControllerTaskUniquePtr ui_controller_task_;
  BatteryMonitorTask battery_monitor_task_;
  SignalMeterTask signal_meter_task_;
  ReceiverSettingsSharedPtr settings_;

  static constexpr uint32_t kIfFrequency = 455000;  // 455 kHz IF
};

}  // namespace receiver_system

#endif  // RECEIVER_MAIN_RECEIVER_SYSTEM_H_
