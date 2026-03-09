#ifndef BUTTON_EVENT_H_
#define BUTTON_EVENT_H_
#include <cstdint>
namespace receiver_system {
struct ButtonEvent {
  enum class Button : uint8_t { kSelect = 0, kPlus = 1, kMinus = 2 };
  enum class Type   : uint8_t { kPress = 0, kRelease = 1, kLongPress = 2 };
  Button button;
  Type   type;
};
}  // namespace receiver_system
#endif
