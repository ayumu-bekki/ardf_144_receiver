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
    ssd1306_(),
    gpio_watch_task_(),
    ui_controller_task_(),
    settings_() {}

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
  ReceiverSettings::InitializeNvs();

  // Load settings from NVS
  settings_.Load();

  // Initialize I2C
  i2c_util::InitializeMaster(hardware_config::kI2cSdaPin,
                             hardware_config::kI2cSclPin);

  // Initialize shared ADC unit
  if (!adc_util::InitializeAdcUnit()) {
    ESP_LOGE(kTag, "Failed to initialize ADC unit");
  }

  si5351a_.Setup(Si5351A::kI2cDefaultAddr);
  mcp4018_.Setup();
  ssd1306_.Setup(SSD1306::kI2cDefaultAddr);

  // I2Cデバイス初期化後、バスが安定するまで待機
  util::SleepMillisecond(100);

  // Initialize Preamp GPIO
  gpio_config_t preamp_gpio_conf = {
      .pin_bit_mask = 1ull << hardware_config::kPreampEnablePin,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&preamp_gpio_conf);

  // Setup GPIO Input Watch Task
  MessageQueue<ButtonEvent>* button_queue =
      ui_controller_task_.GetButtonEventQueue();

  gpio_watch_task_.AddMonitor(
      GpioInputWatchTask::GpioInfo(
          hardware_config::kButtonSelectPin,
          ButtonEvent::Button::kSelect, button_queue),
      GpioInputWatchTask::PULL_UP_REGISTOR_ENABLE);

  gpio_watch_task_.AddMonitor(
      GpioInputWatchTask::GpioInfo(
          hardware_config::kButtonPlusPin,
          ButtonEvent::Button::kPlus, button_queue),
      GpioInputWatchTask::PULL_UP_REGISTOR_ENABLE);

  gpio_watch_task_.AddMonitor(
      GpioInputWatchTask::GpioInfo(
          hardware_config::kButtonMinusPin,
          ButtonEvent::Button::kMinus, button_queue),
      GpioInputWatchTask::PULL_UP_REGISTOR_ENABLE);

  gpio_watch_task_.Start();

  // Start Battery Monitor Task first (performs initial measurement in Initialize)
  battery_monitor_task_.Start();

  // Start Signal Meter Task
  signal_meter_task_.Start();

  // Wait for initial battery measurement and I2C bus to stabilize
  util::SleepMillisecond(500);

  // Initialize UI Controller Task with settings (calls UpdateDisplay)
  ui_controller_task_.InitializeSettings(this, &settings_);

  // Start UI Controller Task
  ui_controller_task_.Start();

  ESP_LOGI(kTag, "Receiver System initialized");

  while (true) {
    util::SleepMillisecond(1000);
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
  const float normalized = static_cast<float>(volume) /
                           hardware_config::kVolumeInputMax;

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

void ReceiverSystem::UpdateDisplay(uint8_t cursor_pos, uint32_t freq_hz,
                                   bool preamp, uint8_t volume,
                                   float battery_voltage,
                                   int signal_strength,
                                   int peak_signal_strength) {
  ESP_LOGD(kTag, "UpdateDisplay: cursor=%u freq=%u preamp=%d vol=%u batt=%.2f sig=%d",
           cursor_pos, freq_hz, preamp, volume, battery_voltage, signal_strength);

  ssd1306_.ClearDisplay();
  ssd1306_.SetContrast(hardware_config::kDisplayContrastDimmed);

  // === Top row: Status bar (Y=0, size=1) ===
  // Format: ">Att  ... [ATT badge] [battery icon]"
  const char* cursor_label = "";
  switch (cursor_pos) {
    case 0: cursor_label = "Fn:Att"; break;
    case 1: cursor_label = "Fn:Vol"; break;
    case 2: cursor_label = "Fn:Frq"; break;
    default: cursor_label = "      "; break;
  }
  ssd1306_.DrawLogoIcon(hardware_config::kHeadphoneIconX,
                        hardware_config::kHeadphoneIconY, true);
  ssd1306_.DrawString(hardware_config::kCursorLabelX,
                      hardware_config::kDisplayLineStatusY,
                      cursor_label, true, 1);

  // ATT badge (white background, black text, rounded corners)
  if (!preamp) {
    ssd1306_.DrawBadge(hardware_config::kAttBadgeX,
                       hardware_config::kAttBadgeY,
                       hardware_config::kAttBadgeWidth,
                       hardware_config::kAttBadgeHeight, "ATT");
  }

  // Battery icon (right side of status bar)
  float batt_ratio = (battery_voltage - hardware_config::kBatteryVoltageMin) /
                     (hardware_config::kBatteryVoltageMax -
                      hardware_config::kBatteryVoltageMin);
  if (batt_ratio < 0.0f) batt_ratio = 0.0f;
  if (batt_ratio > 1.0f) batt_ratio = 1.0f;
  int16_t batt_fill = static_cast<int16_t>(
      batt_ratio * hardware_config::kBatteryFillMax);
  ssd1306_.DrawBatteryIcon(hardware_config::kBatteryIconX,
                           hardware_config::kBatteryIconY,
                           batt_fill, true);

  // === Middle row: Frequency (Y=8, 10x14 large font) ===
  char freq_str[16] = {};
  const uint32_t freq_mhz_int = freq_hz / hardware_config::kFreqToMhzDivisor;
  const uint32_t freq_khz_frac =
      (freq_hz % hardware_config::kFreqToKhzModulo) /
      hardware_config::kFreqToKhzDivisor;
  snprintf(freq_str, sizeof(freq_str), "%3u.%02u MHz",
           freq_mhz_int, freq_khz_frac);
  const int16_t freq_width = ssd1306_.MeasureLargeString(freq_str);
  const int16_t freq_x = (SSD1306::kWidth - freq_width) / 2;
  ssd1306_.DrawLargeString(freq_x, hardware_config::kDisplayLineFreqY,
                           freq_str, true);

  // === Bottom row: S-meter (Y=24, size=1) ===
  // S-value text (left side)
  int s_value = signal_strength / 11;
  if (s_value > 9) s_value = 9;
  char s_str[4] = {};
  snprintf(s_str, sizeof(s_str), "S%d", s_value);
  ssd1306_.DrawString(hardware_config::kSmeterTextX,
                      hardware_config::kDisplayLineSmeterY,
                      s_str, true, 1);

  // S-meter bar outline
  ssd1306_.DrawRect(hardware_config::kSmeterBarX,
                    hardware_config::kSmeterBarY,
                    hardware_config::kSmeterBarWidth,
                    hardware_config::kSmeterBarHeight, true);

  // Fill bar based on signal strength (0-100 -> 0-64px)
  if (signal_strength > 0) {
    int bar_width = signal_strength * hardware_config::kSmeterBarInnerMax / 100;
    if (bar_width > hardware_config::kSmeterBarInnerMax) {
      bar_width = hardware_config::kSmeterBarInnerMax;
    }
    if (bar_width > 0) {
      ssd1306_.FillRect(hardware_config::kSmeterBarX + 1,
                        hardware_config::kSmeterBarY + 1,
                        bar_width,
                        hardware_config::kSmeterBarHeight - 2, true);
    }
  }

  // Draw peak indicator as vertical line
  if (peak_signal_strength > 0) {
    int peak_x = peak_signal_strength * hardware_config::kSmeterBarInnerMax / 100;
    if (peak_x > hardware_config::kSmeterBarInnerMax) {
      peak_x = hardware_config::kSmeterBarInnerMax;
    }
    if (peak_x > 0) {
      ssd1306_.DrawLine(hardware_config::kSmeterBarX + peak_x,
                        hardware_config::kSmeterBarY,
                        hardware_config::kSmeterBarX + peak_x,
                        hardware_config::kSmeterBarY + hardware_config::kSmeterBarHeight - 1,
                        true);
    }
  }

  // === Bottom row: Volume bar (right 1/3) ===
  // Speaker icon
  ssd1306_.DrawSpeakerIcon(hardware_config::kVolumeIconX,
                           hardware_config::kDisplayLineSmeterY, true);
  // Volume bar outline
  ssd1306_.DrawRect(hardware_config::kVolumeBarX,
                    hardware_config::kVolumeBarY,
                    hardware_config::kVolumeBarWidth,
                    hardware_config::kVolumeBarHeight, true);
  // Volume bar fill (volume 0-30 -> 0-28px)
  if (volume > 0) {
    int vol_w = volume * hardware_config::kVolumeBarInnerMax /
                hardware_config::kVolumeMax;
    if (vol_w > hardware_config::kVolumeBarInnerMax) {
      vol_w = hardware_config::kVolumeBarInnerMax;
    }
    if (vol_w > 0) {
      ssd1306_.FillRect(hardware_config::kVolumeBarX + 1,
                        hardware_config::kVolumeBarY + 1,
                        vol_w,
                        hardware_config::kVolumeBarHeight - 2, true);
    }
  }

  ssd1306_.Display();
}

void ReceiverSystem::SetDisplayOn(bool on) {
  ESP_LOGD(kTag, "SetDisplayOn: %s", on ? "ON" : "OFF");

  if (on) {
    ssd1306_.DisplayOn();
  } else {
    ssd1306_.DisplayOff();
  }
}

}  // namespace receiver_system

