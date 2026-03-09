#ifndef GPIO_INPUT_WATCH_TASK_H_
#define GPIO_INPUT_WATCH_TASK_H_
// (C)2024 bekki.jp
// GPIO input watch task

// Include ----------------------
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>

#include <vector>

#include "button_event.h"
#include "gptimer.h"
#include "hardware_config.h"
#include "logger.h"
#include "message_queue.h"
#include "task.h"

namespace receiver_system {

class GpioInputWatchTask final : public Task {
 public:
  static constexpr std::string_view TASK_NAME = "GpioInputWatchTask";
  static constexpr int32_t PRIORITY = Task::PRIORITY_LOW;
  static constexpr int32_t CORE_ID = PRO_CPU_NUM;

 private:
  // Debounce configuration
  static constexpr int32_t EDGE_COUNTER = hardware_config::kDebounceEdgeCount;

  // Long press timing (based on polling interval)
  static constexpr int32_t LONG_PRESS_COUNTER =
      hardware_config::kLongPressThresholdCount;

  static constexpr int32_t LONG_PRESS_REPEAT_INTERVAL =
      hardware_config::kLongPressRepeatIntervalCount;

 public:
  enum GpioPullUpDown {
    NONE,
    PULL_UP_REGISTOR_ENABLE,
    PULL_DOWN_REGISTOR_ENABLE,
  };

  class GpioInfo {
   public:
    enum class Status : int32_t {
      kStatusIdle,
      kStatusPressed,
      kStatusLongPressed,
    };

    GpioInfo(gpio_num_t gpio_no,
             ButtonEvent::Button button_id,
             MessageQueue<ButtonEvent>* event_queue)
        : gpio_no_(gpio_no),
          cnt_(0),
          repeat_counter_(0),
          status_(Status::kStatusIdle),
          button_id_(button_id),
          event_queue_(event_queue) {}

    gpio_num_t GetGpioNo() const { return gpio_no_; }

    void Check() {
      const bool is_pressed = (gpio_get_level(gpio_no_) == 0);

      if (is_pressed) {
        // Button is pressed
        if (status_ == Status::kStatusIdle) {
          // Initial press detection
          ++cnt_;
          if (cnt_ >= EDGE_COUNTER) {
            status_ = Status::kStatusPressed;
            cnt_ = 0;
            if (event_queue_) {
              event_queue_->Send({button_id_, ButtonEvent::Type::kPress});
            }
          }
        } else if (status_ == Status::kStatusPressed) {
          // Check for long press
          ++cnt_;
          if (cnt_ >= LONG_PRESS_COUNTER) {
            status_ = Status::kStatusLongPressed;
            cnt_ = 0;
            repeat_counter_ = 0;
            if (event_queue_) {
              event_queue_->Send({button_id_, ButtonEvent::Type::kLongPress});
            }
          }
        } else if (status_ == Status::kStatusLongPressed) {
          // Repeat during long press
          ++repeat_counter_;
          if (repeat_counter_ >= LONG_PRESS_REPEAT_INTERVAL) {
            repeat_counter_ = 0;
            if (event_queue_) {
              event_queue_->Send({button_id_, ButtonEvent::Type::kLongPress});
            }
          }
        }
      } else {
        // Button is released
        if (status_ != Status::kStatusIdle) {
          if (event_queue_) {
            // Flush stale LongPress events before sending Release,
            // so the UI thread sees Release immediately.
            event_queue_->Reset();
            event_queue_->Send({button_id_, ButtonEvent::Type::kRelease});
          }
          status_ = Status::kStatusIdle;
          cnt_ = 0;
          repeat_counter_ = 0;
        }
      }
    }

   private:
    gpio_num_t gpio_no_;
    int32_t cnt_;
    int32_t repeat_counter_;
    Status status_;
    ButtonEvent::Button button_id_;
    MessageQueue<ButtonEvent>* event_queue_;
  };

 public:
  GpioInputWatchTask()
      : Task(std::string(TASK_NAME).c_str(), PRIORITY, CORE_ID),
        message_queue_(),
        gptimer_(),
        gpio_list_() {
    // Create MessageQueue
    if (!message_queue_.Create(hardware_config::kGpioMessageQueueSize)) {
      ESP_LOGE(kTag, "Creating queue failed");
    }

    // Create Timer
    gptimer_.Create(hardware_config::kGpioTimerResolutionUs,
                    &GpioInputWatchTask::TimerCallback, &message_queue_);
  }

  ~GpioInputWatchTask() {
    gptimer_.Destroy();
    message_queue_.Destroy();
  }

  void Initialize() override {
    ESP_LOGI(kTag, "Start GpioInputWatchTask");
    gpio_install_isr_service(0);
  }

  void Update() override {
    gptimer_.Start(hardware_config::kGpioPollingIntervalUs);
    int event_type = 0;
    while (true) {
      if (message_queue_.ReceiveWait(&event_type,
                                     hardware_config::kGpioQueueTimeoutTicks)) {
        if (event_type == 1) {
          for (auto&& gpio_info : gpio_list_) {
            gpio_info.Check();
          }
        }
      }
    }
  }

  void AddMonitor(GpioInfo gpio_info,
                  GpioPullUpDown gpio_pullupdown = GpioPullUpDown::NONE) {
    // Setting GPIO Input
    // 注意:IO34～IO39は内部プルアップ/プルダウン抵抗は無し
    gpio_config_t io_input_conf = {
        .pin_bit_mask = 1ull << gpio_info.GetGpioNo(),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en =
            (gpio_pullupdown == PULL_UP_REGISTOR_ENABLE ? GPIO_PULLUP_ENABLE
                                                        : GPIO_PULLUP_DISABLE),
        .pull_down_en = (gpio_pullupdown == PULL_DOWN_REGISTOR_ENABLE
                             ? GPIO_PULLDOWN_ENABLE
                             : GPIO_PULLDOWN_DISABLE),
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io_input_conf);

    gpio_list_.emplace_back(std::move(gpio_info));
  }

 private:
  static bool TimerCallback(gptimer_handle_t timer,
                            const gptimer_alarm_event_data_t* event_data,
                            void* message_queue) {
    MessageQueue<int>* const queue =
        static_cast<MessageQueue<int>*>(message_queue);
    return queue->SendFromISR(1);
  }

 private:
  MessageQueue<int> message_queue_;
  GPTimer gptimer_;
  std::vector<GpioInfo> gpio_list_;
};


}  // namespace receiver_system

#endif  // GPIO_INPUT_WATCH_TASK_H_

// EOF
