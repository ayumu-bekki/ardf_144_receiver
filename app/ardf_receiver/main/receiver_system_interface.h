#ifndef BFOX_RECEIVER_MAIN_BFOX_RECEIVER_INTERFACE_H_
#define BFOX_RECEIVER_MAIN_BFOX_RECEIVER_INTERFACE_H_
// ESP32 B-Fox Receiver
// (C)2025 bekki.jp

// Include ----------------------
#include <memory>

namespace receiver_system {

class ReceiverSystemInterface {
 public:
  virtual ~ReceiverSystemInterface() = default;

  // Device control methods
  virtual void SetFrequency(uint32_t freq_hz) = 0;
  virtual void SetPreampEnable(bool enable) = 0;
  virtual void SetVolume(uint8_t volume) = 0;

  // Signal meter data
  virtual int GetSignalStrength() const = 0;
  virtual int GetPeakSignalStrength() const = 0;

  // Battery voltage
  virtual float GetBatteryVoltage() const = 0;

  // Display control methods
  virtual void UpdateDisplay(uint8_t cursor_pos, uint32_t freq_hz,
                            bool preamp, uint8_t volume, float battery_voltage,
                            int signal_strength, int peak_signal_strength) = 0;
  virtual void SetDisplayOn(bool on) = 0;
};

using ReceiverSystemInterfaceSharedPtr = std::shared_ptr<ReceiverSystemInterface>;
using ReceiverSystemInterfaceConstSharedPtr =
    std::shared_ptr<const ReceiverSystemInterface>;
using ReceiverSystemInterfaceWeakPtr = std::weak_ptr<ReceiverSystemInterface>;
using ReceiverSystemInterfaceConstWeakPtr =
    std::weak_ptr<const ReceiverSystemInterface>;

}  // namespace receiver_system

#endif  // BFOX_RECEIVER_MAIN_BFOX_RECEIVER_INTERFACE_H_
