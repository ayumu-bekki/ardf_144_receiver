#ifndef RECEIVER_MAIN_SSD1306_H_
#define RECEIVER_MAIN_SSD1306_H_

#include <driver/i2c_master.h>
#include <esp_log.h>

#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "hardware_config.h"

namespace receiver_system {

class SSD1306 {
 public:
  static constexpr uint8_t kI2cDefaultAddr = 0x3C;

  // ディスプレイサイズ
  static constexpr uint8_t kWidth = 128;
  static constexpr uint8_t kHeight = 32;
  static constexpr uint16_t kBufferSize = (kWidth * kHeight) / 8;

  // コマンド/データ制御バイト
  static constexpr uint8_t kControlCmdSingle = 0x80;
  static constexpr uint8_t kControlCmdStream = 0x00;
  static constexpr uint8_t kControlDataStream = 0x40;

  // SSD1306コマンド
  static constexpr uint8_t kCmdSetContrast = 0x81;
  static constexpr uint8_t kCmdDisplayAllOnResume = 0xA4;
  static constexpr uint8_t kCmdDisplayAllOn = 0xA5;
  static constexpr uint8_t kCmdNormalDisplay = 0xA6;
  static constexpr uint8_t kCmdInvertDisplay = 0xA7;
  static constexpr uint8_t kCmdDisplayOff = 0xAE;
  static constexpr uint8_t kCmdDisplayOn = 0xAF;
  static constexpr uint8_t kCmdSetDisplayOffset = 0xD3;
  static constexpr uint8_t kCmdSetCompins = 0xDA;
  static constexpr uint8_t kCmdSetVcomDetect = 0xDB;
  static constexpr uint8_t kCmdSetDisplayClockDiv = 0xD5;
  static constexpr uint8_t kCmdSetPrecharge = 0xD9;
  static constexpr uint8_t kCmdSetMultiplex = 0xA8;
  static constexpr uint8_t kCmdSetLowColumn = 0x00;
  static constexpr uint8_t kCmdSetHighColumn = 0x10;
  static constexpr uint8_t kCmdSetStartLine = 0x40;
  static constexpr uint8_t kCmdMemoryMode = 0x20;
  static constexpr uint8_t kCmdColumnAddr = 0x21;
  static constexpr uint8_t kCmdPageAddr = 0x22;
  static constexpr uint8_t kCmdComScanInc = 0xC0;
  static constexpr uint8_t kCmdComScanDec = 0xC8;
  static constexpr uint8_t kCmdSegRemap = 0xA0;
  static constexpr uint8_t kCmdChargePump = 0x8D;
  static constexpr uint8_t kCmdExternalVcc = 0x01;
  static constexpr uint8_t kCmdSwitchCapVcc = 0x02;

  // スクロールコマンド
  static constexpr uint8_t kCmdActivateScroll = 0x2F;
  static constexpr uint8_t kCmdDeactivateScroll = 0x2E;
  static constexpr uint8_t kCmdSetVerticalScrollArea = 0xA3;
  static constexpr uint8_t kCmdRightHorizontalScroll = 0x26;
  static constexpr uint8_t kCmdLeftHorizontalScroll = 0x27;
  static constexpr uint8_t kCmdVerticalRightHorizontalScroll = 0x29;
  static constexpr uint8_t kCmdVerticalLeftHorizontalScroll = 0x2A;

 public:
  SSD1306();
  ~SSD1306();

  void Setup(const uint8_t address);
  void ClearDisplay();
  void Display();
  void SetPixel(int16_t x, int16_t y, bool color);
  void DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, bool color);
  void DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, bool color);
  void FillRect(int16_t x, int16_t y, int16_t w, int16_t h, bool color);
  void DrawChar(int16_t x, int16_t y, char c, bool color, uint8_t size);
  void DrawString(int16_t x, int16_t y, const char* str, bool color,
                  uint8_t size);
  void DrawLargeChar(int16_t x, int16_t y, char c, bool color);
  void DrawLargeString(int16_t x, int16_t y, const char* str, bool color);
  int16_t MeasureLargeString(const char* str) const;
  void DrawSpeakerIcon(int16_t x, int16_t y, bool color);
  void DrawBatteryIcon(int16_t x, int16_t y, int16_t fill_level, bool color);
  void DrawBadge(int16_t x, int16_t y);
  void DrawLogoIcon(int16_t x, int16_t y, bool color);
  void SetContrast(uint8_t contrast);
  void InvertDisplay(bool invert);
  void DisplayOn();
  void DisplayOff();

 private:
  void WriteCommand(const uint8_t cmd);
  void WriteData(const uint8_t* data, size_t len);
  void InitializeDisplay();

  // Display initialization parameter values
  static constexpr uint8_t kInitClockDiv = 0xF0;       // High frequency
  static constexpr uint8_t kInitContrast = 0x8F;       // Default contrast
  static constexpr uint8_t kInitPrecharge = 0xF1;      // Precharge period
  static constexpr uint8_t kInitVcomhDeselect = 0x40;  // VCOMH voltage
  static constexpr uint8_t kInitChargePumpOn = 0x14;   // Charge pump enabled
  static constexpr uint8_t kInitCompinsConfig = 0x02;  // 128x32 config
  static constexpr uint8_t kInitMemoryMode = 0x00;     // Horizontal mode
  static constexpr uint8_t kInitSegmentRemap = 0x01;   // Remap enabled

  // Font constants
  static constexpr uint8_t kFontWidth = 5;
  static constexpr uint8_t kFontHeight = 8;
  static constexpr uint8_t kFontCharSpacing = 6;  // 5 + 1 pixel gap
  static constexpr char kFontCharMin = 32;        // Space character
  static constexpr char kFontCharMax = 126;       // ~ character

  // Large font constants (10x14, for frequency display)
  static constexpr uint8_t kLargeFontWidth = 10;
  static constexpr uint8_t kLargeFontHeight = 14;
  static constexpr uint8_t kLargeFontCharSpacing = 11;  // 10 + 1 pixel gap

  // Data transfer constants
  static constexpr size_t kDataChunkSize =
      hardware_config::kDisplayDataChunkSize;

  // Pixel manipulation bit constants
  static constexpr uint8_t kPixelBitMask = 0x07;  // (y & 7) - 3-bit mask
  static constexpr uint8_t kPixelByteShift = 3;   // y / 8 = y >> 3
  static constexpr uint8_t kPageEndAddr = 3;      // (kHeight/8) - 1
  static constexpr uint8_t kColumnEndAddr = 127;  // kWidth - 1

 private:
  i2c_master_dev_handle_t dev_handle_;
  uint8_t i2c_address_;
  std::vector<uint8_t> buffer_;
  std::vector<uint8_t> prev_buffer_;
};

using SSD1306SharedPtr = std::shared_ptr<SSD1306>;
using SSD1306WeakPtr = std::weak_ptr<SSD1306>;

}  // namespace receiver_system

#endif  // RECEIVER_MAIN_SSD1306_H_
