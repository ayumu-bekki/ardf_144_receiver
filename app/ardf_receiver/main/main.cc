// ReceiverSystem
// (C)2025 bekki.jp

// Include ----------------------
#include "receiver_system.h"

/// Entry Point
extern "C" void app_main() {
  std::make_shared<receiver_system::ReceiverSystem>()->Start();
}
