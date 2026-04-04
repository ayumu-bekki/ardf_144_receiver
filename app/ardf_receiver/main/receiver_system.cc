// ESP32 B-Fox Receiver
// (C)2025 bekki.jp

#include "receiver_system.h"

#include "adc_util.h"
#include "button_event.h"
#include "driver/gpio.h"
#include "hardware_config.h"
#include "i2c_util.h"
#include "logger.h"
#include "util.h"
#include "xiao_esp32c6_pin.h"

namespace receiver_system {

// Unused GPIO Pins
// Used pins: GPIO0(A0/Battery), GPIO1(A1/SMeter), GPIO16(D6/Preamp),
//            GPIO18(D10/BtnMinus), GPIO19(D8/BtnSelect), GPIO20(D9/BtnPlus),
//            GPIO22(D4/SDA), GPIO23(D5/SCL)
const gpio_num_t kUnusedGpioPins[] = {
    xiao_esp32c6_pin::kD2,             // GPIO2 (A2)
    xiao_esp32c6_pin::kWifiEnable,     // GPIO3
    xiao_esp32c6_pin::kMtms,           // GPIO4 (A4)
    xiao_esp32c6_pin::kMtdi,           // GPIO5 (A5)
    xiao_esp32c6_pin::kMtck,           // GPIO6 (A6)
    xiao_esp32c6_pin::kMtdo,           // GPIO7
    xiao_esp32c6_pin::kBoot,           // GPIO9
    xiao_esp32c6_pin::kWifiAntConfig,  // GPIO14
    xiao_esp32c6_pin::kLedBuiltin,     // GPIO15
    xiao_esp32c6_pin::kRx,             // GPIO17 (D7)
    xiao_esp32c6_pin::kSS,             // GPIO21 (D3)
};

constexpr i2c_port_t kI2cPortNo = hardware_config::kI2cPort;

ReceiverSystem::ReceiverSystem()
    : si5351a_(),
      mcp4018_(),
      ssd1306_(std::make_shared<SSD1306>()),
      gpio_watch_task_(),
      ui_controller_task_(nullptr),
      settings_(std::make_shared<ReceiverSettings>()) {}

ReceiverSystem::~ReceiverSystem() = default;

void ReceiverSystem::Start() {
  // Initialize Log
  logger::InitializeLogLevel();

  ESP_LOGI(kTag, "Startup Receiver System");

  // Set unused GPIOs to input with pull-up for stability
  for (const auto& gpio_num : kUnusedGpioPins) {
    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
    gpio_set_pull_mode(gpio_num, GPIO_PULLDOWN_ONLY);
  }

  // Initialize NVS
  util::InitializeNvs();

  // Load settings from NVS
  settings_->Load();

  // Initialize I2C
  i2c_util::InitializeMaster(hardware_config::kI2cSdaPin,
                             hardware_config::kI2cSclPin);

  // Initialize shared ADC unit
  if (!adc_util::InitializeAdcUnit()) {
    ESP_LOGE(kTag, "Failed to initialize ADC unit");
  }

  // Wait for I2C devices to be ready after power-on before first access.
  // Si5351A requires time for internal crystal oscillator to stabilize.
  util::SleepMillisecond(hardware_config::kI2cBusStabilizeDelayMs);

  si5351a_.Setup(Si5351A::kI2cDefaultAddr);
  mcp4018_.Setup();
  ssd1306_->Setup(SSD1306::kI2cDefaultAddr);

  // Initialize Preamp GPIO
  gpio_config_t preamp_gpio_conf = {
      .pin_bit_mask = 1ull << hardware_config::kPreampEnablePin,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&preamp_gpio_conf);

  // Construct UI Controller Task first to obtain the button event queue.
  // Constructor loads settings, applies to devices, and calls UpdateDisplay().
  ui_controller_task_ = std::make_unique<UiControllerTask>(
      weak_from_this(), 
      settings_,
      ssd1306_);

  // Setup GPIO Input Watch Task with the button event queue
  MessageQueue<ButtonEvent>* button_queue =
      ui_controller_task_->GetButtonEventQueue();

  gpio_watch_task_.AddMonitor(
      GpioInputWatchTask::GpioInfo(hardware_config::kButtonSelectPin,
                                   ButtonEvent::Button::kSelect, button_queue),
      GpioInputWatchTask::PULL_UP_REGISTOR_ENABLE);

  gpio_watch_task_.AddMonitor(
      GpioInputWatchTask::GpioInfo(hardware_config::kButtonPlusPin,
                                   ButtonEvent::Button::kPlus, button_queue),
      GpioInputWatchTask::PULL_UP_REGISTOR_ENABLE);

  gpio_watch_task_.AddMonitor(
      GpioInputWatchTask::GpioInfo(hardware_config::kButtonMinusPin,
                                   ButtonEvent::Button::kMinus, button_queue),
      GpioInputWatchTask::PULL_UP_REGISTOR_ENABLE);

  gpio_watch_task_.Start();

  // Start Battery Monitor Task
  battery_monitor_task_.Start();

  // Start Signal Meter Task
  signal_meter_task_.Start();

  // Start UI Controller Task
  ui_controller_task_->Start();

  ESP_LOGI(kTag, "Receiver System initialized");

  while (true) {
    util::SleepMillisecond(hardware_config::kMainLoopSleepMs);
  }
}

void ReceiverSystem::SetFrequency(uint32_t freq_hz) {
  ESP_LOGD(kTag, "SetFrequency: %u Hz", freq_hz);

  // Add IF frequency (455 kHz) for Si5351A
  si5351a_.SetFrequency(Si5351A::Clk::kClk0, freq_hz + kIfFrequency);
}

void ReceiverSystem::SetPreampEnable(bool enable) {
  ESP_LOGD(kTag, "SetPreampEnable: %s", enable ? "Enable" : "Disable");

  // Set preamp GPIO HIGH for enable, LOW for disable
  gpio_set_level(hardware_config::kPreampEnablePin, enable ? 1 : 0);
}

void ReceiverSystem::SetVolume(uint8_t volume) {
  ESP_LOGD(kTag, "SetVolume: %u", volume);

  // Convert volume (0-30) to wiper value (0-127) with A-curve (cubic)
  const float normalized =
      static_cast<float>(volume) / hardware_config::kVolumeInputMax;

  // Apply A-curve: raise to power of kVolumeCurveExponent
  float a_curve = normalized;
  for (int i = 1; i < hardware_config::kVolumeCurveExponent; ++i) {
    a_curve *= normalized;
  }

  const uint8_t wiper_value =
      static_cast<uint8_t>(a_curve * hardware_config::kVolumeWiperMax);

  mcp4018_.SetWiper(wiper_value);
}

int ReceiverSystem::GetSignalStrength() const {
  return signal_meter_task_.GetSmeterRaw();
}

int ReceiverSystem::GetPeakSignalStrength() const {
  return signal_meter_task_.GetPeakSmeterRaw();
}

float ReceiverSystem::GetBatteryVoltage() const {
  return battery_monitor_task_.GetBatteryVoltage();
}

}  // namespace receiver_system
