#ifndef RECEIVER_MAIN_MCP4018_H_
#define RECEIVER_MAIN_MCP4018_H_

#include <driver/i2c_master.h>
#include <esp_log.h>

#include <cstdint>
#include <memory>

namespace receiver_system {

class MCP4018 {
 public:
  static constexpr uint8_t kI2cDefaultAddr = 0x2F;

  // ワイパー設定範囲: 0-127
  // 注意: 0 = 最大抵抗値 (10kΩ), 127 = 最小抵抗値 (~80Ω)
  static constexpr uint8_t kWiperMin = 0;
  static constexpr uint8_t kWiperMax = 127;

 public:
  MCP4018();
  ~MCP4018();

  void Setup();

  // ワイパー位置を設定 (0-127)
  // value: ワイパー位置 (0 = 最大抵抗値, 127 = 最小抵抗値)
  // 注意: MCP4018では0が最大抵抗、127が最小抵抗となります
  //       抵抗値 R = (127 - value) × Rstep + Rwiper
  //       Rstep ≈ 78.43Ω (10kΩ ÷ 127.5)
  //       Rwiper ≈ 80Ω (ワイパー抵抗)
  void SetWiper(uint8_t value);

 private:
  void Write(const uint8_t value);

 private:
  i2c_master_dev_handle_t dev_handle_;
};

using MCP4018UniquePtr = std::unique_ptr<MCP4018>;
using MCP4018SharedPtr = std::shared_ptr<MCP4018>;

}  // namespace receiver_system

#endif  // RECEIVER_MAIN_MCP4018_H_
