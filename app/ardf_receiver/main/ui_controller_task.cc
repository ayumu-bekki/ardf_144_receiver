// UI Controller Task
// (C)2025 bekki.jp

#include "ui_controller_task.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "button_event.h"
#include "hardware_config.h"
#include "logger.h"
#include "receiver_settings.h"
#include "receiver_system_interface.h"

namespace receiver_system {

UiControllerTask::UiControllerTask()
    : Task(std::string(TASK_NAME).c_str(), PRIORITY, CORE_ID),
      event_queue_(),
      interface_(nullptr),
      settings_(nullptr),
      current_menu_item_(kMenuPreamp),
      frequency_hz_(kFrequencyDefault),
      preamp_enabled_(kPreampDefault),
      volume_(kVolumeDefault),
      settings_dirty_(false) {
  if (!event_queue_.Create(hardware_config::kButtonEventQueueSize)) {
    ESP_LOGE(kTag, "Creating button event queue failed");
  }
}

UiControllerTask::~UiControllerTask() = default;

void UiControllerTask::Initialize() {
  ESP_LOGI(kTag, "UiControllerTask started");
}

void UiControllerTask::InitializeSettings(ReceiverSystemInterface* interface,
                                          ReceiverSettings* settings) {
  interface_ = interface;
  settings_ = settings;

  // Load saved settings
  if (settings_) {
    frequency_hz_ = settings_->GetFrequencyHz();
    preamp_enabled_ = settings_->GetPreampEnabled();
    volume_ = settings_->GetVolume();
  }

  // Apply settings to devices
  if (interface_) {
    interface_->SetFrequency(frequency_hz_);
    interface_->SetPreampEnable(preamp_enabled_);
    interface_->SetVolume(volume_);
    UpdateDisplay();
  }
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
  using Type   = ButtonEvent::Type;
  switch (event.button) {
    case Button::kSelect:
      if (event.type == Type::kPress) OnButtonSelect();
      break;
    case Button::kPlus:
      if      (event.type == Type::kPress)     OnButtonPlus();
      else if (event.type == Type::kLongPress) OnButtonPlusLongPress();
      else if (event.type == Type::kRelease)   OnButtonPlusRelease();
      break;
    case Button::kMinus:
      if      (event.type == Type::kPress)     OnButtonMinus();
      else if (event.type == Type::kLongPress) OnButtonMinusLongPress();
      else if (event.type == Type::kRelease)   OnButtonMinusRelease();
      break;
  }
}

void UiControllerTask::OnButtonSelect() {
  // Move cursor to next menu item
  current_menu_item_ = static_cast<MenuItem>(
      (current_menu_item_ + 1) % kMenuItemCount);

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

void UiControllerTask::UpdateFrequency(int32_t delta_10khz, bool save_immediately) {
  int64_t delta = static_cast<int64_t>(delta_10khz) * static_cast<int64_t>(kFrequencyStep);
  int64_t new_freq = static_cast<int64_t>(frequency_hz_) + delta;

  // Clamp to valid range
  if (new_freq < static_cast<int64_t>(kFrequencyMin)) {
    new_freq = kFrequencyMin;
  } else if (new_freq > static_cast<int64_t>(kFrequencyMax)) {
    new_freq = kFrequencyMax;
  }

  frequency_hz_ = static_cast<uint32_t>(new_freq);

  if (interface_) {
    interface_->SetFrequency(frequency_hz_);
    UpdateDisplay();
  }

  if (save_immediately) {
    // Save immediately to NVS for single press
    if (settings_) {
      settings_->SaveFrequency(frequency_hz_);
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

  if (interface_) {
    interface_->SetPreampEnable(preamp_enabled_);
    UpdateDisplay();
  }

  // Save immediately to NVS
  if (settings_) {
    settings_->SavePreamp(preamp_enabled_);
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

  if (interface_) {
    interface_->SetVolume(volume_);
    UpdateDisplay();
  }

  if (save_immediately) {
    // Save immediately to NVS for single press
    if (settings_) {
      settings_->SaveVolume(volume_);
    }
  } else {
    // Mark as dirty for later save on button release
    settings_dirty_ = true;
  }
}

void UiControllerTask::UpdateDisplay() {
  if (interface_) {
    float battery_voltage = interface_->GetBatteryVoltage();
    int signal_strength = interface_->GetSignalStrength();
    int peak_signal_strength = interface_->GetPeakSignalStrength();
    interface_->UpdateDisplay(static_cast<uint8_t>(current_menu_item_),
                             frequency_hz_, preamp_enabled_, volume_,
                             battery_voltage, signal_strength,
                             peak_signal_strength);
  }
}

void UiControllerTask::SaveCurrentSettings() {
  if (settings_dirty_ && settings_) {
    ESP_LOGD(kTag, "Saving settings to NVS");
    settings_->SaveFrequency(frequency_hz_);
    settings_->SaveVolume(volume_);
    settings_dirty_ = false;
  }
}

}  // namespace receiver_system
