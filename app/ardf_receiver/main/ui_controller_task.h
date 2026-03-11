#ifndef RECEIVER_MAIN_UI_CONTROLLER_TASK_H_
#define RECEIVER_MAIN_UI_CONTROLLER_TASK_H_
// UI Controller Task
// (C)2025 bekki.jp

// Include ----------------------
#include <cstdint>
#include <memory>
#include <string_view>

#include "button_event.h"
#include "message_queue.h"
#include "receiver_settings.h"
#include "receiver_system_interface.h"
#include "ssd1306.h"
#include "task.h"

namespace receiver_system {

/// UI Controller Task for menu selection and value change
class UiControllerTask final : public Task {
 public:
  static constexpr std::string_view TASK_NAME = "UiControllerTask";
  static constexpr int32_t PRIORITY = Task::PRIORITY_NORMAL;
  static constexpr int32_t CORE_ID = PRO_CPU_NUM;

  enum MenuItem {
    kMenuPreamp = 0,
    kMenuVolume = 1,
    kMenuFrequency = 2,
    kMenuItemCount = 3
  };

 private:
  static constexpr uint32_t kFrequencyMin = 140000000;      // 140.00 MHz
  static constexpr uint32_t kFrequencyMax = 150000000;      // 150.00 MHz
  static constexpr uint32_t kFrequencyDefault = 145000000;  // 145.00 MHz
  static constexpr uint32_t kFrequencyStep = 10000;         // 0.01 MHz = 10 kHz
  static constexpr uint8_t kVolumeMin = 0;
  static constexpr uint8_t kVolumeMax = 30;
  static constexpr uint8_t kVolumeDefault = 15;
  static constexpr bool kPreampDefault = true;  // Default enabled

 public:
  UiControllerTask(ReceiverSystemInterfaceWeakPtr interface,
                   ReceiverSettingsWeakPtr settings,
                   SSD1306WeakPtr display);
  ~UiControllerTask();

  void Initialize() override;
  void Update() override;

  MessageQueue<ButtonEvent>* GetButtonEventQueue() { return &event_queue_; }

  // Getters
  MenuItem GetCurrentMenuItem() const { return current_menu_item_; }
  uint32_t GetFrequencyHz() const { return frequency_hz_; }
  bool GetPreampEnabled() const { return preamp_enabled_; }
  uint8_t GetVolume() const { return volume_; }

 private:
  void DispatchEvent(const ButtonEvent& event);
  void OnButtonSelect();
  void OnButtonPlus();
  void OnButtonMinus();
  void OnButtonPlusLongPress();
  void OnButtonMinusLongPress();
  void OnButtonPlusRelease();
  void OnButtonMinusRelease();
  void UpdateFrequency(int32_t delta_10khz, bool save_immediately = true);
  void UpdatePreamp(bool enable);
  void UpdateVolume(int32_t delta, bool save_immediately = true);
  void UpdateDisplay();
  void SaveCurrentSettings();

 private:
  MessageQueue<ButtonEvent> event_queue_;
  ReceiverSystemInterfaceWeakPtr interface_;
  ReceiverSettingsWeakPtr settings_;
  SSD1306WeakPtr display_;
  MenuItem current_menu_item_;
  uint32_t frequency_hz_;  // Frequency in Hz
  bool preamp_enabled_;    // Preamp ON/OFF
  uint8_t volume_;         // Volume 0-30
  bool settings_dirty_;    // Flag to indicate unsaved changes
};

using UiControllerTaskUniquePtr = std::unique_ptr<UiControllerTask>;

}  // namespace receiver_system

#endif  // RECEIVER_MAIN_UI_CONTROLLER_TASK_H_
