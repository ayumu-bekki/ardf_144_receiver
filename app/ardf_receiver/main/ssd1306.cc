#include "ssd1306.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rom/ets_sys.h>

#include <algorithm>
#include <memory>

#include "i2c_util.h"
#include "logger.h"

namespace receiver_system {

// 5x7フォントデータ（ASCII 32-126）
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00},  // ' '
    {0x00, 0x00, 0x5F, 0x00, 0x00},  // '!'
    {0x00, 0x07, 0x00, 0x07, 0x00},  // '"'
    {0x14, 0x7F, 0x14, 0x7F, 0x14},  // '#'
    {0x24, 0x2A, 0x7F, 0x2A, 0x12},  // '$'
    {0x23, 0x13, 0x08, 0x64, 0x62},  // '%'
    {0x36, 0x49, 0x55, 0x22, 0x50},  // '&'
    {0x00, 0x05, 0x03, 0x00, 0x00},  // '''
    {0x00, 0x1C, 0x22, 0x41, 0x00},  // '('
    {0x00, 0x41, 0x22, 0x1C, 0x00},  // ')'
    {0x14, 0x08, 0x3E, 0x08, 0x14},  // '*'
    {0x08, 0x08, 0x3E, 0x08, 0x08},  // '+'
    {0x00, 0x50, 0x30, 0x00, 0x00},  // ','
    {0x08, 0x08, 0x08, 0x08, 0x08},  // '-'
    {0x00, 0x60, 0x60, 0x00, 0x00},  // '.'
    {0x20, 0x10, 0x08, 0x04, 0x02},  // '/'
    {0x3E, 0x51, 0x49, 0x45, 0x3E},  // '0'
    {0x00, 0x42, 0x7F, 0x40, 0x00},  // '1'
    {0x42, 0x61, 0x51, 0x49, 0x46},  // '2'
    {0x21, 0x41, 0x45, 0x4B, 0x31},  // '3'
    {0x18, 0x14, 0x12, 0x7F, 0x10},  // '4'
    {0x27, 0x45, 0x45, 0x45, 0x39},  // '5'
    {0x3C, 0x4A, 0x49, 0x49, 0x30},  // '6'
    {0x01, 0x71, 0x09, 0x05, 0x03},  // '7'
    {0x36, 0x49, 0x49, 0x49, 0x36},  // '8'
    {0x06, 0x49, 0x49, 0x29, 0x1E},  // '9'
    {0x00, 0x36, 0x36, 0x00, 0x00},  // ':'
    {0x00, 0x56, 0x36, 0x00, 0x00},  // ';'
    {0x08, 0x14, 0x22, 0x41, 0x00},  // '<'
    {0x14, 0x14, 0x14, 0x14, 0x14},  // '='
    {0x00, 0x41, 0x22, 0x14, 0x08},  // '>'
    {0x02, 0x01, 0x51, 0x09, 0x06},  // '?'
    {0x32, 0x49, 0x79, 0x41, 0x3E},  // '@'
    {0x7E, 0x11, 0x11, 0x11, 0x7E},  // 'A'
    {0x7F, 0x49, 0x49, 0x49, 0x36},  // 'B'
    {0x3E, 0x41, 0x41, 0x41, 0x22},  // 'C'
    {0x7F, 0x41, 0x41, 0x22, 0x1C},  // 'D'
    {0x7F, 0x49, 0x49, 0x49, 0x41},  // 'E'
    {0x7F, 0x09, 0x09, 0x09, 0x01},  // 'F'
    {0x3E, 0x41, 0x49, 0x49, 0x7A},  // 'G'
    {0x7F, 0x08, 0x08, 0x08, 0x7F},  // 'H'
    {0x00, 0x41, 0x7F, 0x41, 0x00},  // 'I'
    {0x20, 0x40, 0x41, 0x3F, 0x01},  // 'J'
    {0x7F, 0x08, 0x14, 0x22, 0x41},  // 'K'
    {0x7F, 0x40, 0x40, 0x40, 0x40},  // 'L'
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},  // 'M'
    {0x7F, 0x04, 0x08, 0x10, 0x7F},  // 'N'
    {0x3E, 0x41, 0x41, 0x41, 0x3E},  // 'O'
    {0x7F, 0x09, 0x09, 0x09, 0x06},  // 'P'
    {0x3E, 0x41, 0x51, 0x21, 0x5E},  // 'Q'
    {0x7F, 0x09, 0x19, 0x29, 0x46},  // 'R'
    {0x46, 0x49, 0x49, 0x49, 0x31},  // 'S'
    {0x01, 0x01, 0x7F, 0x01, 0x01},  // 'T'
    {0x3F, 0x40, 0x40, 0x40, 0x3F},  // 'U'
    {0x1F, 0x20, 0x40, 0x20, 0x1F},  // 'V'
    {0x3F, 0x40, 0x38, 0x40, 0x3F},  // 'W'
    {0x63, 0x14, 0x08, 0x14, 0x63},  // 'X'
    {0x07, 0x08, 0x70, 0x08, 0x07},  // 'Y'
    {0x61, 0x51, 0x49, 0x45, 0x43},  // 'Z'
    {0x00, 0x7F, 0x41, 0x41, 0x00},  // '['
    {0x02, 0x04, 0x08, 0x10, 0x20},  // '\'
    {0x00, 0x41, 0x41, 0x7F, 0x00},  // ']'
    {0x04, 0x02, 0x01, 0x02, 0x04},  // '^'
    {0x40, 0x40, 0x40, 0x40, 0x40},  // '_'
    {0x00, 0x01, 0x02, 0x04, 0x00},  // '`'
    {0x20, 0x54, 0x54, 0x54, 0x78},  // 'a'
    {0x7F, 0x48, 0x44, 0x44, 0x38},  // 'b'
    {0x38, 0x44, 0x44, 0x44, 0x20},  // 'c'
    {0x38, 0x44, 0x44, 0x48, 0x7F},  // 'd'
    {0x38, 0x54, 0x54, 0x54, 0x18},  // 'e'
    {0x08, 0x7E, 0x09, 0x01, 0x02},  // 'f'
    {0x0C, 0x52, 0x52, 0x52, 0x3E},  // 'g'
    {0x7F, 0x08, 0x04, 0x04, 0x78},  // 'h'
    {0x00, 0x44, 0x7D, 0x40, 0x00},  // 'i'
    {0x20, 0x40, 0x44, 0x3D, 0x00},  // 'j'
    {0x7F, 0x10, 0x28, 0x44, 0x00},  // 'k'
    {0x00, 0x41, 0x7F, 0x40, 0x00},  // 'l'
    {0x7C, 0x04, 0x18, 0x04, 0x78},  // 'm'
    {0x7C, 0x08, 0x04, 0x04, 0x78},  // 'n'
    {0x38, 0x44, 0x44, 0x44, 0x38},  // 'o'
    {0x7C, 0x14, 0x14, 0x14, 0x08},  // 'p'
    {0x08, 0x14, 0x14, 0x18, 0x7C},  // 'q'
    {0x7C, 0x08, 0x04, 0x04, 0x08},  // 'r'
    {0x48, 0x54, 0x54, 0x54, 0x20},  // 's'
    {0x04, 0x3F, 0x44, 0x40, 0x20},  // 't'
    {0x3C, 0x40, 0x40, 0x20, 0x7C},  // 'u'
    {0x1C, 0x20, 0x40, 0x20, 0x1C},  // 'v'
    {0x3C, 0x40, 0x30, 0x40, 0x3C},  // 'w'
    {0x44, 0x28, 0x10, 0x28, 0x44},  // 'x'
    {0x0C, 0x50, 0x50, 0x50, 0x3C},  // 'y'
    {0x44, 0x64, 0x54, 0x4C, 0x44},  // 'z'
    {0x00, 0x08, 0x36, 0x41, 0x00},  // '{'
    {0x00, 0x00, 0x7F, 0x00, 0x00},  // '|'
    {0x00, 0x41, 0x36, 0x08, 0x00},  // '}'
    {0x10, 0x08, 0x08, 0x10, 0x08},  // '~'
};

SSD1306::SSD1306()
    : dev_handle_(nullptr), i2c_address_(0), buffer_(), prev_buffer_() {}

SSD1306::~SSD1306() {
  if (dev_handle_) {
    i2c_master_bus_rm_device(dev_handle_);
    dev_handle_ = nullptr;
  }
}

void SSD1306::Setup(const uint8_t address) {
  i2c_address_ = address;

  // Get the I2C bus handle
  i2c_master_bus_handle_t bus_handle = i2c_util::GetBusHandle();
  if (!bus_handle) {
    ESP_LOGE(kTag, "I2C bus not initialized");
    return;
  }

  // Add device to the bus
  i2c_device_config_t dev_config = {};
  dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_config.device_address = address;
  dev_config.scl_speed_hz = i2c_util::kI2cMasterFrequencyHz;

  esp_err_t ret =
      i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "Failed to add SSD1306 device: %s", esp_err_to_name(ret));
    return;
  }

  // バッファを確保
  buffer_.resize(kBufferSize, 0x00);
  prev_buffer_.resize(kBufferSize, 0xFF);  // Force first display

  if (buffer_.empty() || prev_buffer_.empty()) {
    ESP_LOGE(kTag, "Failed to allocate display buffers");
    return;
  }

  InitializeDisplay();
}

void SSD1306::InitializeDisplay() {
  // 初期化シーケンス（SSD1306データシート準拠）
  WriteCommand(kCmdDisplayOff);

  WriteCommand(kCmdSetDisplayClockDiv);
  WriteCommand(kInitClockDiv);  // 高周波設定（可聴域ノイズ回避）

  WriteCommand(kCmdSetMultiplex);
  WriteCommand(kHeight - 1);  // 32-1=31

  WriteCommand(kCmdSetDisplayOffset);
  WriteCommand(0x00);  // オフセットなし

  WriteCommand(kCmdSetStartLine | 0x00);  // スタートライン = 0

  WriteCommand(kCmdChargePump);
  WriteCommand(kInitChargePumpOn);  // チャージポンプを有効化

  WriteCommand(kCmdMemoryMode);
  WriteCommand(kInitMemoryMode);  // 水平アドレッシングモード

  WriteCommand(kCmdSegRemap | kInitSegmentRemap);  // セグメントリマップ

  WriteCommand(kCmdComScanDec);  // COM出力スキャン方向

  WriteCommand(kCmdSetCompins);
  WriteCommand(kInitCompinsConfig);  // 128x32用の設定

  WriteCommand(kCmdSetContrast);
  WriteCommand(kInitContrast);  // コントラスト

  WriteCommand(kCmdSetPrecharge);
  WriteCommand(kInitPrecharge);  // プリチャージ期間

  WriteCommand(kCmdSetVcomDetect);
  WriteCommand(kInitVcomhDeselect);  // VCOMH電圧

  WriteCommand(kCmdDisplayAllOnResume);
  WriteCommand(kCmdNormalDisplay);
  WriteCommand(kCmdDisplayOn);

  ESP_LOGI(kTag, "SSD1306 initialized (128x32)");
}

void SSD1306::ClearDisplay() {
  std::fill(buffer_.begin(), buffer_.end(), 0x00);
}

void SSD1306::Display() {
  // Skip I2C transfer if buffer unchanged
  if (buffer_ == prev_buffer_) {
    return;
  }
  WriteCommand(kCmdColumnAddr);
  WriteCommand(0);
  WriteCommand(kColumnEndAddr);

  WriteCommand(kCmdPageAddr);
  WriteCommand(0);
  WriteCommand(kPageEndAddr);

  WriteData(buffer_.data(), kBufferSize);
  prev_buffer_ = buffer_;
}

void SSD1306::SetPixel(int16_t x, int16_t y, bool color) {
  if (x < 0 || x >= kWidth || y < 0 || y >= kHeight) {
    return;
  }

  if (color) {
    buffer_[x + (y >> kPixelByteShift) * kWidth] |= (1 << (y & kPixelBitMask));
  } else {
    buffer_[x + (y >> kPixelByteShift) * kWidth] &= ~(1 << (y & kPixelBitMask));
  }
}

void SSD1306::DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                       bool color) {
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }

  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  int16_t dx = x1 - x0;
  int16_t dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep = (y0 < y1) ? 1 : -1;

  for (; x0 <= x1; x0++) {
    if (steep) {
      SetPixel(y0, x0, color);
    } else {
      SetPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

void SSD1306::DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, bool color) {
  DrawLine(x, y, x + w - 1, y, color);
  DrawLine(x + w - 1, y, x + w - 1, y + h - 1, color);
  DrawLine(x + w - 1, y + h - 1, x, y + h - 1, color);
  DrawLine(x, y + h - 1, x, y, color);
}

void SSD1306::FillRect(int16_t x, int16_t y, int16_t w, int16_t h, bool color) {
  for (int16_t i = x; i < x + w; i++) {
    for (int16_t j = y; j < y + h; j++) {
      SetPixel(i, j, color);
    }
  }
}

void SSD1306::DrawChar(int16_t x, int16_t y, char c, bool color, uint8_t size) {
  if (c < kFontCharMin || c > kFontCharMax) {
    c = kFontCharMin;  // 範囲外は空白に
  }

  const uint8_t* glyph = font5x7[c - kFontCharMin];

  for (uint8_t i = 0; i < kFontWidth; i++) {
    uint8_t line = glyph[i];
    for (uint8_t j = 0; j < kFontHeight; j++) {
      if (line & 0x01) {
        if (size == 1) {
          SetPixel(x + i, y + j, color);
        } else {
          FillRect(x + i * size, y + j * size, size, size, color);
        }
      }
      line >>= 1;
    }
  }
}

void SSD1306::DrawString(int16_t x, int16_t y, const char* str, bool color,
                         uint8_t size) {
  int16_t cursor_x = x;
  int16_t cursor_y = y;

  while (*str) {
    if (*str == '\n') {
      cursor_y += kFontHeight * size;
      cursor_x = x;
    } else if (*str == '\r') {
      // 無視
    } else {
      DrawChar(cursor_x, cursor_y, *str, color, size);
      cursor_x += kFontCharSpacing * size;
      if (cursor_x + kFontWidth * size >= kWidth) {
        cursor_x = x;
        cursor_y += kFontHeight * size;
      }
    }
    str++;
  }
}

// 10x14 large font bitmap data (column-major, 2 bytes per column)
// Covers: ' ' '.' '0'-'9' 'H' 'M' 'z'
// Each glyph is 10 columns x 14 rows, stored as 10 x uint16_t (LSB = top row)
//
// Design: rows 0-13, 2px stroke, symmetric top/bottom curves
//
// '0':                  '1':
//  0: ..XXXXXX..        0: ....XX....
//  1: .XXXXXXXX.        1: ...XXX....
//  2: XX......XX        2: ..XXXX....
//  3: XX......XX        3: ....XX....
//  4: XX......XX        4: ....XX....
//  5: XX......XX        5: ....XX....
//  6: XX......XX        6: ....XX....
//  7: XX......XX        7: ....XX....
//  8: XX......XX        8: ....XX....
//  9: XX......XX        9: ....XX....
// 10: XX......XX       10: ....XX....
// 11: XX......XX       11: ....XX....
// 12: .XXXXXXXX.       12: .XXXXXXXX.
// 13: ..XXXXXX..       13: ..XXXXXX..
// '0':  ..XXXXXX..   '1':  ....XX....
//       .XXXXXXXX.         ...XXX....
//       XX......XX         ..XXXX....
//       XX......XX         ....XX....
//       XX......XX         ....XX....
//       XX......XX         ....XX....
//       XX......XX         ....XX....
//       XX......XX         ....XX....
//       XX......XX         ....XX....
//       XX......XX         ....XX....
//       XX......XX         ....XX....
//       XX......XX         ....XX....
//       .XXXXXXXX.         .XXXXXXXX.
//       ..XXXXXX..         ..XXXXXX..
static const uint16_t font10x14_0[] = {
    0x0FFC, 0x1FFE, 0x3003, 0x3003, 0x3003,
    0x3003, 0x3003, 0x3003, 0x1FFE, 0x0FFC,
};
static const uint16_t font10x14_1[] = {
    0x0000, 0x1000, 0x3004, 0x3006, 0x3FFF,
    0x3FFF, 0x3000, 0x3000, 0x1000, 0x0000,
};
// '2':  ..XXXXXX..   '3':  ..XXXXXX..
//       .XXXXXXXX.         .XXXXXXXX.
//       XX......XX         XX......XX
//       ........XX         ........XX
//       ........XX         ........XX
//       .......XX.         ...XXXXXX.
//       ......XX..         ...XXXXXX.
//       .....XX...         ........XX
//       ....XX....         ........XX
//       ...XX.....         ........XX
//       ..XX......         XX......XX
//       XX........         XX......XX
//       XXXXXXXXXX         .XXXXXXXX.
//       XXXXXXXXXX         ..XXXXXX..
static const uint16_t font10x14_2[] = {
    0x3804, 0x3806, 0x3403, 0x3603, 0x3303,
    0x3183, 0x30C3, 0x3063, 0x303E, 0x301C,
};
static const uint16_t font10x14_3[] = {
    0x0C04, 0x1C06, 0x3003, 0x3063, 0x3063,
    0x3063, 0x3063, 0x3063, 0x1FFE, 0x0F9C,
};
// '4':  ......XX..   '5':  XXXXXXXXXX
//       .....XXX..         XXXXXXXXXX
//       ....XXXX..         XX........
//       ...XX.XX..         XX........
//       ..XX..XX..         XXXXXXXX..
//       .XX...XX..         XXXXXXXXX.
//       XX....XX..         ........XX
//       XX....XX..         ........XX
//       XXXXXXXXXX         ........XX
//       XXXXXXXXXX         ........XX
//       ......XX..         XX......XX
//       ......XX..         XX......XX
//       ......XX..         .XXXXXXXX.
//       ......XX..         ..XXXXXX..
static const uint16_t font10x14_4[] = {
    0x03C0, 0x03E0, 0x0330, 0x0318, 0x030C,
    0x0306, 0x3FFF, 0x3FFF, 0x0300, 0x0300,
};
static const uint16_t font10x14_5[] = {
    0x0C3F, 0x1C3F, 0x3033, 0x3033, 0x3033,
    0x3033, 0x3033, 0x3033, 0x1FE3, 0x0FC3,
};
// '6':  ..XXXXXX..   '7':  XXXXXXXXXX
//       .XXXXXXXX.         XXXXXXXXXX
//       XX......XX         ........XX
//       XX........         .......XX.
//       XX........         ......XX..
//       XXXXXXXX..         .....XX...
//       XXXXXXXXX.         ....XX....
//       XX......XX         ....XX....
//       XX......XX         ...XX.....
//       XX......XX         ...XX.....
//       XX......XX         ..XX......
//       XX......XX         ..XX......
//       .XXXXXXXX.         ..XX......
//       ..XXXXXX..         ..XX......
static const uint16_t font10x14_6[] = {
    0x0FFC, 0x1FFE, 0x3063, 0x3063, 0x3063,
    0x3063, 0x3063, 0x3063, 0x1FC6, 0x0F84,
};
static const uint16_t font10x14_7[] = {
    0x0003, 0x0003, 0x3C03, 0x3F03, 0x03C3,
    0x00E3, 0x0033, 0x001B, 0x000F, 0x0007,
};
// '8':  ..XXXXXX..   '9':  ..XXXXXX..
//       .XXXXXXXX.         .XXXXXXXX.
//       XX......XX         XX......XX
//       XX......XX         XX......XX
//       XX......XX         XX......XX
//       .XXXXXXXX.         XX......XX
//       .XXXXXXXX.         .XXXXXXXXX
//       XX......XX         ..XXXXXXXX
//       XX......XX         ........XX
//       XX......XX         ........XX
//       XX......XX         XX......XX
//       XX......XX         XX......XX
//       .XXXXXXXX.         .XXXXXXXX.
//       ..XXXXXX..         ..XXXXXX..
static const uint16_t font10x14_8[] = {
    0x0F9C, 0x1FFE, 0x3063, 0x3063, 0x3063,
    0x3063, 0x3063, 0x3063, 0x1FFE, 0x0F9C,
};
static const uint16_t font10x14_9[] = {
    0x0C3C, 0x1C7E, 0x30C3, 0x30C3, 0x30C3,
    0x30C3, 0x30C3, 0x30C3, 0x1FFE, 0x0FFC,
};
// '.'  (rows 12-13 only)
static const uint16_t font10x14_dot[] = {
    0x0000, 0x0000, 0x3000, 0x3000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
};
// 'M':  XX......XX   'H':  XX......XX
//       XXX....XXX         XX......XX
//       XXXX..XXXX         XX......XX
//       XX.XXXX.XX         XX......XX
//       XX..XX..XX         XX......XX
//       XX......XX         XXXXXXXXXX
//       XX......XX         XXXXXXXXXX
//       XX......XX         XX......XX
//       XX......XX         XX......XX
//       XX......XX         XX......XX
//       XX......XX         XX......XX
//       XX......XX         XX......XX
//       XX......XX         XX......XX
//       XX......XX         XX......XX
static const uint16_t font10x14_M[] = {
    0x3FFF, 0x3FFF, 0x0006, 0x000C, 0x0018,
    0x0018, 0x000C, 0x0006, 0x3FFF, 0x3FFF,
};
static const uint16_t font10x14_H[] = {
    0x3FFF, 0x3FFF, 0x0060, 0x0060, 0x0060,
    0x0060, 0x0060, 0x0060, 0x3FFF, 0x3FFF,
};
// 'z' (lowercase, rows 5-13):
//       XXXXXXXX..
//       XXXXXXXX..
//       ......XX..
//       .....XX...
//       ....XX....
//       ...XX.....
//       ..XX......
//       XXXXXXXX..
//       XXXXXXXX..
static const uint16_t font10x14_z[] = {
    0x3060, 0x3060, 0x3860, 0x3C60, 0x3660,
    0x3360, 0x31E0, 0x30E0, 0x0000, 0x0000,
};
static const uint16_t font10x14_space[] = {
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
};

static const uint16_t* GetLargeFontGlyph(char c) {
  switch (c) {
    case '0':
      return font10x14_0;
    case '1':
      return font10x14_1;
    case '2':
      return font10x14_2;
    case '3':
      return font10x14_3;
    case '4':
      return font10x14_4;
    case '5':
      return font10x14_5;
    case '6':
      return font10x14_6;
    case '7':
      return font10x14_7;
    case '8':
      return font10x14_8;
    case '9':
      return font10x14_9;
    case '.':
      return font10x14_dot;
    case 'M':
      return font10x14_M;
    case 'H':
      return font10x14_H;
    case 'z':
      return font10x14_z;
    default:
      return font10x14_space;
  }
}

void SSD1306::DrawLargeChar(int16_t x, int16_t y, char c, bool color) {
  const uint16_t* glyph = GetLargeFontGlyph(c);

  for (uint8_t col = 0; col < kLargeFontWidth; col++) {
    uint16_t column_data = glyph[col];
    for (uint8_t row = 0; row < kLargeFontHeight; row++) {
      if (column_data & (1 << row)) {
        SetPixel(x + col, y + row, color);
      }
    }
  }
}

void SSD1306::DrawLargeString(int16_t x, int16_t y, const char* str,
                              bool color) {
  int16_t cursor_x = x;

  while (*str) {
    if (*str == ' ') {
      cursor_x += kLargeFontCharSpacing / 2;  // Half-width space
    } else {
      DrawLargeChar(cursor_x, y, *str, color);
      cursor_x += kLargeFontCharSpacing;
    }
    str++;
  }
}

int16_t SSD1306::MeasureLargeString(const char* str) const {
  int16_t width = 0;
  while (*str) {
    if (*str == ' ') {
      width += kLargeFontCharSpacing / 2;
    } else {
      width += kLargeFontCharSpacing;
    }
    str++;
  }
  // Remove trailing gap after last non-space character
  if (width > 0) {
    width -= 1;  // Last character doesn't need the 1px gap
  }
  return width;
}

// 7x7 speaker icon bitmap (column-major, LSB = top row)
// Body (rect) + cone (triangle) + sound arc
//        0123456
// Row 0: ....X..
// Row 1: ...XX..
// Row 2: XXXXX.X
// Row 3: XXXXX.X
// Row 4: XXXXX.X
// Row 5: ...XX..
// Row 6: ....X..
static constexpr uint8_t kSpeakerIconWidth = 7;
static constexpr uint8_t kSpeakerIconHeight = 7;
static const uint8_t kSpeakerIcon[7] = {
    0x1C,  // col 0: ..XXX..
    0x1C,  // col 1: ..XXX..
    0x1C,  // col 2: ..XXX..
    0x3E,  // col 3: .XXXXX.
    0x7F,  // col 4: XXXXXXX
    0x00,  // col 5: .......
    0x1C,  // col 6: ..XXX..
};

void SSD1306::DrawSpeakerIcon(int16_t x, int16_t y, bool color) {
  for (uint8_t col = 0; col < kSpeakerIconWidth; col++) {
    uint8_t column_data = kSpeakerIcon[col];
    for (uint8_t row = 0; row < kSpeakerIconHeight; row++) {
      if (column_data & (1 << row)) {
        SetPixel(x + col, y + row, color);
      }
    }
  }
}

void SSD1306::DrawBatteryIcon(int16_t x, int16_t y, int16_t fill_level,
                              bool color) {
  using namespace hardware_config;
  // Terminal nub (left side, vertically centered)
  FillRect(x, y + (kBatteryBodyHeight - kBatteryTerminalHeight) / 2,
           kBatteryTerminalWidth, kBatteryTerminalHeight, color);
  // Body outline (right of terminal)
  DrawRect(x + kBatteryTerminalWidth, y, kBatteryBodyWidth, kBatteryBodyHeight,
           color);
  // Fill level inside body
  if (fill_level > 0) {
    if (fill_level > kBatteryFillMax) {
      fill_level = kBatteryFillMax;
    }
    // Fill from right to left (terminal side = empty, right side = full)
    int16_t fill_x =
        x + kBatteryTerminalWidth + 1 + (kBatteryFillMax - fill_level);
    FillRect(fill_x, y + 1, fill_level, kBatteryBodyHeight - 2, color);
  }
}

// Logo icon: 7x7 bitmap (column-major, LSB = top row)
// Fox/cat ear icon (8x8)
//        0 1 2 3 4 5 6 7
// Row 0: . X . . . . X .
// Row 1: X X . . . . X X
// Row 2: X X X . . X X X
// Row 3: X . X X X X . X
// Row 4: X . . X X . . X
// Row 5: X . X . . X . X   (eyes)
// Row 6: X X X . . X X X   (eyes)
// Row 7: . X X X X X X .
static const uint8_t kLogoIcon[8] = {
    0x7E,  // col 0: rows 1-6
    0xC7,  // col 1: rows 0-2,6-7
    0x8C,  // col 2: rows 2-3,7
    0x98,  // col 3: rows 3-4,7
    0x98,  // col 4: rows 3-4,7
    0xEC,  // col 5: rows 2-3,5-7
    0xE7,  // col 6: rows 0-2,5-7
    0x7E,  // col 7: rows 1-6
};

void SSD1306::DrawLogoIcon(int16_t x, int16_t y, bool color) {
  for (uint8_t col = 0; col < 8; col++) {
    uint8_t column_data = kLogoIcon[col];
    for (uint8_t row = 0; row < 8; row++) {
      if (column_data & (1 << row)) {
        SetPixel(x + col, y + row, color);
      }
    }
  }
}

// ATT badge: 20x7 pre-rendered bitmap (column-major, LSB = top row)
// White background with rounded corners, black "ATT" text cutout
// Layout: border|pad|A(4px)|gap|T(5px)|gap|T(5px)|pad|border
//
// Row 0: .XXXXXXXXXXXXXXXXXX.  (top border, rounded corners)
// Row 1: XX.XX.XXXXX.XXXXX.XX  (white bg, A/T/T row1 cutout)
// Row 2: XX..XXXXXXXXXXXXXXXX  (A sides cutout)
// Row 3: XXXXXXXXXX.XXXXX.XXX  (A mid-bar open)
// Row 4: XX..XXXXXXXXXXXXXXXX
// Row 5: XX..XXXXXXXXXXXXXXXX
// Row 6: .XXXXXXXXXXXXXXXXXX.  (bottom border, rounded corners)
static constexpr uint8_t kAttBadgeWidth = 20;
static constexpr uint8_t kAttBadgeHeight = 7;
static const uint8_t kAttBadgeBitmap[20] = {
    //       row: 6543210
    0x3E,  // col  0: .XXXXX.  left border (corners cleared)
    0x7F,  // col  1: XXXXXXX  left padding
    0x43,  // col  2: XX....X  A col0: rows 2-5 cutout (X..X, XXXX, X..X, X..X)
    0x75,  // col  3: X.X.XXX  A col1: rows 1,3 cutout (.XX.)
    0x75,  // col  4: X.X.XXX  A col2: rows 1,3 cutout (.XX.)
    0x43,  // col  5: XX....X  A col3: rows 2-5 cutout
    0x7F,  // col  6: XXXXXXX  gap
    0x7D,  // col  7: X.XXXXX  T col0: row 1 cutout (top bar)
    0x7D,  // col  8: X.XXXXX  T col1: row 1 cutout
    0x41,  // col  9: X.....X  T col2: rows 1-5 cutout (stem)
    0x7D,  // col 10: X.XXXXX  T col3: row 1 cutout
    0x7D,  // col 11: X.XXXXX  T col4: row 1 cutout
    0x7F,  // col 12: XXXXXXX  gap
    0x7D,  // col 13: X.XXXXX  T col0: row 1 cutout
    0x7D,  // col 14: X.XXXXX  T col1: row 1 cutout
    0x41,  // col 15: X.....X  T col2: rows 1-5 cutout (stem)
    0x7D,  // col 16: X.XXXXX  T col3: row 1 cutout
    0x7D,  // col 17: X.XXXXX  T col4: row 1 cutout
    0x7F,  // col 18: XXXXXXX  right padding
    0x3E,  // col 19: .XXXXX.  right border (corners cleared)
};

void SSD1306::DrawBadge(int16_t x, int16_t y) {
  for (uint8_t col = 0; col < kAttBadgeWidth; col++) {
    uint8_t column_data = kAttBadgeBitmap[col];
    for (uint8_t row = 0; row < kAttBadgeHeight; row++) {
      if (column_data & (1 << row)) {
        SetPixel(x + col, y + row, true);
      }
    }
  }
}

void SSD1306::SetContrast(uint8_t contrast) {
  WriteCommand(kCmdSetContrast);
  WriteCommand(contrast);
}

void SSD1306::InvertDisplay(bool invert) {
  WriteCommand(invert ? kCmdInvertDisplay : kCmdNormalDisplay);
}

void SSD1306::DisplayOn() { WriteCommand(kCmdDisplayOn); }

void SSD1306::DisplayOff() { WriteCommand(kCmdDisplayOff); }

void SSD1306::WriteCommand(const uint8_t cmd) {
  if (!dev_handle_) {
    ESP_LOGE(kTag, "Device handle not initialized");
    return;
  }

  uint8_t write_buf[2] = {kControlCmdStream, cmd};
  esp_err_t ret = i2c_master_transmit(dev_handle_, write_buf, sizeof(write_buf),
                                      i2c_util::kI2cTimeoutMs);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "SSD1306 cmd write failed (cmd=0x%02X): %s", cmd,
             esp_err_to_name(ret));
    i2c_master_bus_handle_t bus = i2c_util::GetBusHandle();
    if (bus) {
      i2c_master_bus_reset(bus);
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
}

void SSD1306::WriteData(const uint8_t* data, size_t len) {
  if (!dev_handle_) {
    ESP_LOGE(kTag, "Device handle not initialized");
    return;
  }

  // Send data in chunks to avoid buffer overflow
  for (size_t i = 0; i < len; i += kDataChunkSize) {
    size_t remaining = len - i;
    size_t current_chunk =
        (remaining > kDataChunkSize) ? kDataChunkSize : remaining;

    // Prepare buffer with control byte + data
    uint8_t write_buf[kDataChunkSize + 1];
    write_buf[0] = kControlDataStream;
    memcpy(&write_buf[1], &data[i], current_chunk);

    esp_err_t ret = i2c_master_transmit(
        dev_handle_, write_buf, current_chunk + 1, i2c_util::kI2cTimeoutMs);
    if (ret != ESP_OK) {
      ESP_LOGE(kTag, "SSD1306 data write failed at offset %zu len %zu: %s", i,
               current_chunk, esp_err_to_name(ret));
      i2c_master_bus_handle_t bus = i2c_util::GetBusHandle();
      if (bus) {
        i2c_master_bus_reset(bus);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      break;
    }
  }
}

}  // namespace receiver_system
