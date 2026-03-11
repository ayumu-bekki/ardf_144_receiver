#ifndef RECEIVER_MAIN_SI5351A_H_
#define RECEIVER_MAIN_SI5351A_H_

#include <driver/i2c_master.h>
#include <esp_log.h>

#include <cstring>
#include <memory>
#include <string>

namespace receiver_system {

class Si5351A {
 public:
  static constexpr uint8_t kI2cDefaultAddr = 0x60;

  // 周波数設定
  static constexpr uint32_t kXtalFreq = 25000000;     // 水晶発振器: 25MHz
  static constexpr uint32_t kPllFreqMin = 600000000;  // PLL周波数最小: 600MHz
  static constexpr uint32_t kPllFreqMax = 900000000;  // PLL周波数最大: 900MHz

  // MultiSynth分周比の制約
  // 整数モード: 6～2048 (レジスタ制限)
  // 分数モード: 8 + 1/1048575 ～ 900
  static constexpr uint32_t kMultiSynthDivMin = 4;
  static constexpr uint32_t kMultiSynthDivMax = 2048;

  // R分周器の最大値 (1, 2, 4, 8, 16, 32, 64, 128)
  static constexpr uint8_t kRDivMax = 0x80;
  static constexpr uint8_t kRDivBitsMax =
      7;  // R分周器のレジスタ値の最大 (2^7 = 128)

  // 分周比探索の閾値
  // この周波数以上の場合、偶数分周比のみを試す（計算高速化のため）
  static constexpr uint32_t kEvenDivOnlyThreshold = 100000;  // 100kHz

  // レジスタアドレス
  static constexpr uint8_t kRegOutputCtrl = 0x03;
  static constexpr uint8_t kRegXtalLoad = 0xb7;
  static constexpr uint8_t kRegClk0Ctrl = 0x10;
  static constexpr uint8_t kRegClk1Ctrl = 0x11;
  static constexpr uint8_t kRegClk2Ctrl = 0x12;
  static constexpr uint8_t kRegPllA = 0x1a;
  static constexpr uint8_t kRegPllB = 0x22;
  static constexpr uint8_t kRegMs0 = 0x2a;
  static constexpr uint8_t kRegMs1 = 0x32;
  static constexpr uint8_t kRegMs2 = 0x3a;
  static constexpr uint8_t kRegPllReset = 0xb1;

  // CLK Control Register values (Integer mode enabled)
  // bit 7: Power Down (0 = enabled, 1 = disabled)
  // bit 6: MS Integer Mode (1 = integer, 0 = fractional)
  // bit 5: MS Source (0 = PLLA, 1 = PLLB)
  // bit 4: CLK Invert (0 = not inverted, 1 = inverted)
  // bit 3-2: CLK_SRC (11 = MS_SRC, 10 = MS_SRC, 01 = CLKIN, 00 = XTAL)
  // bit 1-0: CLK_IDRV Drive Strength (11=8mA, 10=6mA, 01=4mA, 00=2mA)
  // 0x4F = 0b01001111: Power Up, Integer Mode, PLLA, Not Inverted, MultiSynth,
  // 8mA
  static constexpr uint8_t kClkPower8mA =
      0x4F;  // Integer mode + PLLA + MultiSynth + 8mA
  static constexpr uint8_t kClkPower6mA =
      0x4E;  // Integer mode + PLLA + MultiSynth + 6mA
  static constexpr uint8_t kClkPower4mA =
      0x4D;  // Integer mode + PLLA + MultiSynth + 4mA
  static constexpr uint8_t kClkPower2mA =
      0x4C;  // Integer mode + PLLA + MultiSynth + 2mA

  static constexpr uint8_t kResetPllA = 0x20;
  static constexpr uint8_t kResetPllB = 0x80;

  // PLL分数分周器の分母 (20bit最大分解能)
  static constexpr uint32_t kPllDenom = 0x000fffff;

  // MultiSynth レジスタアドレスオフセット (CLKごとに8バイト)
  static constexpr uint8_t kMultiSynthRegStride = 0x08;

  // P1 計算オフセット: P1 = 128*a + floor(128*b/c) - 512
  static constexpr uint32_t kP1Offset = 512;

  // DIVBY4 制御ビット: MSx_DIVBY4[1:0] = 11b (bit 3-2)
  static constexpr uint8_t kDivBy4Bits = 0x0C;

  // 全出力無効化マスク
  static constexpr uint8_t kAllOutputDisabled = 0xFF;

  // CLK パワーダウンビット
  static constexpr uint8_t kClkPowerDown = 0x80;

  enum class Clk {
    kClk0 = 0,
    kClk1,
    kClk2,
  };

 public:
  Si5351A();
  ~Si5351A();

  void Setup(const uint8_t address);
  void SetFrequency(const Clk clk, const uint32_t freq);

 private:
  void Write(const uint8_t reg, const uint8_t value);

  void SetPLL(const uint8_t pll, const uint8_t mult, const uint32_t num,
              const uint32_t denom);
  void SetMultiSynth(const uint8_t synth, const uint32_t div,
                     const uint8_t rdiv_bits);

 private:
  i2c_master_dev_handle_t dev_handle_;
  uint8_t i2c_address_;

  uint8_t clk_output_status_;
};

using Si5351AUniquePtr = std::unique_ptr<Si5351A>;
using Si5351ASharedPtr = std::shared_ptr<Si5351A>;

}  // namespace receiver_system

#endif  // RECEIVER_MAIN_SI5351A_H_
