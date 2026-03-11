// UI Controller Task
// (C)2025 bekki.jp

#include "ui_controller_task.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "button_event.h"
#include "hardware_config.h"
#include "logger.h"
#include "receiver_settings.h"
#include "ssd1306.h"

namespace receiver_system {

UiControllerTask::UiControllerTask(ReceiverSystemInterfaceWeakPtr interface,
                                   ReceiverSettingsWeakPtr settings,
                                   SSD1306WeakPtr display)
    : Task(std::string(TASK_NAME).c_str(), PRIORITY, CORE_ID),
      event_queue_(),
      interface_(std::move(interface)),
      settings_(std::move(settings)),
      display_(std::move(display)),
      current_menu_item_(kMenuPreamp),
      frequency_hz_(kFrequencyDefault),
      preamp_enabled_(kPreampDefault),
      volume_(kVolumeDefault),
      settings_dirty_(false) {
  if (!event_queue_.Create(hardware_config::kButtonEventQueueSize)) {
    ESP_LOGE(kTag, "Creating button event queue failed");
  }

  // Load saved settings
  if (auto s = settings_.lock()) {
    frequency_hz_ = s->GetFrequencyHz();
    preamp_enabled_ = s->GetPreampEnabled();
    volume_ = s->GetVolume();
  }

  // Apply settings to devices
  if (auto iface = interface_.lock()) {
    iface->SetFrequency(frequency_hz_);
    iface->SetPreampEnable(preamp_enabled_);
    iface->SetVolume(volume_);
    UpdateDisplay();
  }
}

UiControllerTask::~UiControllerTask() = default;

void UiControllerTask::Initialize() {
  ESP_LOGI(kTag, "UiControllerTask started");
}

void UiControllerTask::Update() {
  while (true) {
    ButtonEvent event{};
    if (event_queue_.ReceiveWait(&event,
                                 hardware_config::kDisplayUpdateIntervalMs)) {
      // Drain the queue, but prioritize Release: if we see a Release,
      // discard all preceding stale LongPress events and process Release first.
      ButtonEvent pending = event;
      ButtonEvent next{};
      while (event_queue_.ReceiveNonBlock(&next)) {
        if (next.type == ButtonEvent::Type::kRelease) {
          // Discard the accumulated LongPress events, process Release now
          pending = next;
          break;
        }
        DispatchEvent(pending);
        pending = next;
      }
      DispatchEvent(pending);
    }
    UpdateDisplay();
  }
}

void UiControllerTask::DispatchEvent(const ButtonEvent& event) {
  using Button = ButtonEvent::Button;
  using Type = ButtonEvent::Type;
  if (event.button == Button::kSelect) {
    if (event.type == Type::kPress) {
      OnButtonSelect();
    }
  } else if (event.button == Button::kPlus) {
    if (event.type == Type::kPress) {
      OnButtonPlus();
    } else if (event.type == Type::kLongPress) {
      OnButtonPlusLongPress();
    } else if (event.type == Type::kRelease) {
        OnButtonPlusRelease();
    }
  } else if (event.button == Button::kMinus) {
    if (event.type == Type::kPress) {
      OnButtonMinus();
    } else if (event.type == Type::kLongPress) {
      OnButtonMinusLongPress();
    } else if (event.type == Type::kRelease) {
      OnButtonMinusRelease();
    }
  }
}

void UiControllerTask::OnButtonSelect() {
  // Move cursor to next menu item
  current_menu_item_ =
      static_cast<MenuItem>((current_menu_item_ + 1) % kMenuItemCount);

  UpdateDisplay();
}

void UiControllerTask::OnButtonPlus() {
  if (current_menu_item_ == kMenuFrequency) {
    UpdateFrequency(+1);
  } else if (current_menu_item_ == kMenuPreamp) {
    UpdatePreamp(!preamp_enabled_);
  } else if (current_menu_item_ == kMenuVolume) {
    UpdateVolume(+1);
  }
}

void UiControllerTask::OnButtonMinus() {
  if (current_menu_item_ == kMenuFrequency) {
    UpdateFrequency(-1);
  } else if (current_menu_item_ == kMenuPreamp) {
    UpdatePreamp(!preamp_enabled_);
  } else if (current_menu_item_ == kMenuVolume) {
    UpdateVolume(-1);
  }
}

void UiControllerTask::OnButtonPlusLongPress() {
  if (current_menu_item_ == kMenuFrequency) {
    UpdateFrequency(+1, false);  // Don't save immediately during long press
  } else if (current_menu_item_ == kMenuVolume) {
    UpdateVolume(+1, false);  // Don't save immediately during long press
  }
}

void UiControllerTask::OnButtonMinusLongPress() {
  if (current_menu_item_ == kMenuFrequency) {
    UpdateFrequency(-1, false);  // Don't save immediately during long press
  } else if (current_menu_item_ == kMenuVolume) {
    UpdateVolume(-1, false);  // Don't save immediately during long press
  }
}

void UiControllerTask::OnButtonPlusRelease() {
  SaveCurrentSettings();
}

void UiControllerTask::OnButtonMinusRelease() {
  SaveCurrentSettings();
}

void UiControllerTask::UpdateFrequency(int32_t delta_10khz,
                                       bool save_immediately) {
  int64_t delta =
      static_cast<int64_t>(delta_10khz) * static_cast<int64_t>(kFrequencyStep);
  int64_t new_freq = static_cast<int64_t>(frequency_hz_) + delta;

  // Clamp to valid range
  if (new_freq < static_cast<int64_t>(kFrequencyMin)) {
    new_freq = kFrequencyMin;
  } else if (new_freq > static_cast<int64_t>(kFrequencyMax)) {
    new_freq = kFrequencyMax;
  }

  frequency_hz_ = static_cast<uint32_t>(new_freq);

  if (auto iface = interface_.lock()) {
    iface->SetFrequency(frequency_hz_);
    UpdateDisplay();
  }

  if (save_immediately) {
    // Save immediately to NVS for single press
    if (auto s = settings_.lock()) {
      s->SaveFrequency(frequency_hz_);
    }
  } else {
    // Mark as dirty for later save on button release
    settings_dirty_ = true;
  }
}

void UiControllerTask::UpdatePreamp(bool enable) {
  if (preamp_enabled_ == enable) {
    return;  // No change
  }

  preamp_enabled_ = enable;

  if (auto iface = interface_.lock()) {
    iface->SetPreampEnable(preamp_enabled_);
    UpdateDisplay();
  }

  // Save immediately to NVS
  if (auto s = settings_.lock()) {
    s->SavePreamp(preamp_enabled_);
  }
}

void UiControllerTask::UpdateVolume(int32_t delta, bool save_immediately) {
  int32_t new_volume = static_cast<int32_t>(volume_) + delta;

  // Clamp to valid range
  if (new_volume < kVolumeMin) {
    new_volume = kVolumeMin;
  } else if (new_volume > kVolumeMax) {
    new_volume = kVolumeMax;
  }

  volume_ = static_cast<uint8_t>(new_volume);

  if (auto iface = interface_.lock()) {
    iface->SetVolume(volume_);
    UpdateDisplay();
  }

  if (save_immediately) {
    // Save immediately to NVS for single press
    if (auto s = settings_.lock()) {
      s->SaveVolume(volume_);
    }
  } else {
    // Mark as dirty for later save on button release
    settings_dirty_ = true;
  }
}

void UiControllerTask::UpdateDisplay() {
  auto display = display_.lock();
  if (!display) {
    return;
  }

  float battery_voltage = 0.0f;
  int signal_strength = 0;
  int peak_signal_strength = 0;
  if (auto iface = interface_.lock()) {
    battery_voltage = iface->GetBatteryVoltage();
    signal_strength = iface->GetSignalStrength();
    peak_signal_strength = iface->GetPeakSignalStrength();
  }

  ESP_LOGD(kTag,
           "UpdateDisplay: cursor=%u freq=%u preamp=%d vol=%u batt=%.2f sig=%d",
           static_cast<uint8_t>(current_menu_item_), frequency_hz_,
           preamp_enabled_, volume_, battery_voltage, signal_strength);

  display->ClearDisplay();
  display->SetContrast(hardware_config::kDisplayContrastDimmed);

  // === Top row: Status bar ===
  const char* cursor_label = "";
  if (current_menu_item_ == kMenuPreamp) {
    cursor_label = "Fn:Att";
  } else if (current_menu_item_ == kMenuVolume) {
    cursor_label = "Fn:Vol";
  } else if (current_menu_item_ == kMenuFrequency) {
    cursor_label = "Fn:Frq";
  } else {
      cursor_label = "      ";
  }
  display->DrawLogoIcon(hardware_config::kHeadphoneIconX,
                        hardware_config::kHeadphoneIconY, true);
  display->DrawString(hardware_config::kCursorLabelX,
                      hardware_config::kDisplayLineStatusY, cursor_label, true,
                      1);

  // ATT badge (white background, black text, rounded corners)
  if (!preamp_enabled_) {
    display->DrawBadge(hardware_config::kAttBadgeX,
                       hardware_config::kAttBadgeY);
  }

  // Battery icon (right side of status bar)
  float batt_ratio = (battery_voltage - hardware_config::kBatteryVoltageMin) /
                     (hardware_config::kBatteryVoltageMax -
                      hardware_config::kBatteryVoltageMin);
  if (batt_ratio < 0.0f) {
    batt_ratio = 0.0f;
  }
  if (batt_ratio > 1.0f) {
    batt_ratio = 1.0f;
  }
  int16_t batt_fill =
      static_cast<int16_t>(batt_ratio * hardware_config::kBatteryFillMax);
  display->DrawBatteryIcon(hardware_config::kBatteryIconX,
                           hardware_config::kBatteryIconY, batt_fill, true);

  // === Middle row: Frequency ===
  char freq_str[16] = {};
  const uint32_t freq_mhz_int = frequency_hz_ / hardware_config::kFreqToMhzDivisor;
  const uint32_t freq_khz_frac =
      (frequency_hz_ % hardware_config::kFreqToKhzModulo) /
      hardware_config::kFreqToKhzDivisor;
  snprintf(freq_str, sizeof(freq_str), "%3u.%02u MHz", freq_mhz_int,
           freq_khz_frac);
  const int16_t freq_width = display->MeasureLargeString(freq_str);
  const int16_t freq_x = (SSD1306::kWidth - freq_width) / 2;
  display->DrawLargeString(freq_x, hardware_config::kDisplayLineFreqY, freq_str,
                           true);

  // === Bottom row: S-meter ===
  int s_value = signal_strength / hardware_config::kSmeterSValueDivisor;
  if (s_value > hardware_config::kSmeterSValueMax) {
    s_value = hardware_config::kSmeterSValueMax;
  }
  char s_str[4] = {};
  snprintf(s_str, sizeof(s_str), "S%d", s_value);
  display->DrawString(hardware_config::kSmeterTextX,
                      hardware_config::kDisplayLineSmeterY, s_str, true, 1);

  display->DrawRect(hardware_config::kSmeterBarX, hardware_config::kSmeterBarY,
                    hardware_config::kSmeterBarWidth,
                    hardware_config::kSmeterBarHeight, true);

  if (signal_strength > 0) {
    int bar_width = signal_strength * hardware_config::kSmeterBarInnerMax /
                    hardware_config::kSmeterPercentMax;
    if (bar_width > hardware_config::kSmeterBarInnerMax) {
      bar_width = hardware_config::kSmeterBarInnerMax;
    }
    if (bar_width > 0) {
      display->FillRect(hardware_config::kSmeterBarX + 1,
                        hardware_config::kSmeterBarY + 1, bar_width,
                        hardware_config::kSmeterBarHeight - 2, true);
    }
  }

  if (peak_signal_strength > 0) {
    int peak_x = peak_signal_strength * hardware_config::kSmeterBarInnerMax /
                 hardware_config::kSmeterPercentMax;
    if (peak_x > hardware_config::kSmeterBarInnerMax) {
      peak_x = hardware_config::kSmeterBarInnerMax;
    }
    if (peak_x > 0) {
      display->DrawLine(
          hardware_config::kSmeterBarX + peak_x, hardware_config::kSmeterBarY,
          hardware_config::kSmeterBarX + peak_x,
          hardware_config::kSmeterBarY + hardware_config::kSmeterBarHeight - 1,
          true);
    }
  }

  // === Bottom row: Volume bar ===
  display->DrawSpeakerIcon(hardware_config::kVolumeIconX,
                           hardware_config::kDisplayLineSmeterY, true);
  display->DrawRect(hardware_config::kVolumeBarX, hardware_config::kVolumeBarY,
                    hardware_config::kVolumeBarWidth,
                    hardware_config::kVolumeBarHeight, true);
  if (volume_ > 0) {
    int vol_w = volume_ * hardware_config::kVolumeBarInnerMax /
                hardware_config::kVolumeMax;
    if (vol_w > hardware_config::kVolumeBarInnerMax) {
      vol_w = hardware_config::kVolumeBarInnerMax;
    }
    if (vol_w > 0) {
      display->FillRect(hardware_config::kVolumeBarX + 1,
                        hardware_config::kVolumeBarY + 1, vol_w,
                        hardware_config::kVolumeBarHeight - 2, true);
    }
  }

  display->Display();
}

void UiControllerTask::SaveCurrentSettings() {
  if (settings_dirty_) {
    if (auto s = settings_.lock()) {
      ESP_LOGD(kTag, "Saving settings to NVS");
      s->SaveFrequency(frequency_hz_);
      s->SaveVolume(volume_);
      settings_dirty_ = false;
    }
  }
}

}  // namespace receiver_system
