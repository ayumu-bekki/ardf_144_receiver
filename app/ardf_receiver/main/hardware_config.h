#ifndef HARDWARE_CONFIG_H_
#define HARDWARE_CONFIG_H_
// ESP32 B-Fox Receiver
// (C)2025 bekki.jp
// Hardware Configuration Constants

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "xiao_esp32c6_pin.h"

namespace receiver_system {
namespace hardware_config {

// ============================================================================
// GPIO Pin Assignments (Application-specific mappings)
// ============================================================================

// I2C Bus Pins
constexpr gpio_num_t kI2cSdaPin = xiao_esp32c6_pin::kSda;
constexpr gpio_num_t kI2cSclPin = xiao_esp32c6_pin::kScl;

// Preamp Control
constexpr gpio_num_t kPreampEnablePin = xiao_esp32c6_pin::kD6;

// Button Inputs
constexpr gpio_num_t kButtonSelectPin = xiao_esp32c6_pin::kD10; 
constexpr gpio_num_t kButtonPlusPin = xiao_esp32c6_pin::kD9;
constexpr gpio_num_t kButtonMinusPin = xiao_esp32c6_pin::kD8;

// ADC Input
constexpr gpio_num_t kBatteryVoltagePin = xiao_esp32c6_pin::kA0;
constexpr gpio_num_t kSignalStrengthMeterPin = xiao_esp32c6_pin::kA1;

// ============================================================================
// Battery Monitoring Configuration
// ============================================================================

// Battery voltage monitoring interval (milliseconds)
constexpr uint32_t kBatteryMonitorIntervalMs = 30000;  // 30 seconds

// Number of ADC samples to average per measurement
constexpr int kBatteryAdcSampleCount = 5;

// Voltage divider circuit constants
// Battery voltage is divided by R1 and R2
// V_adc = V_battery × (R2 / (R1 + R2))
constexpr float kBatteryVoltageDividerR1 = 200000.0f;  // 200kΩ (battery side)
constexpr float kBatteryVoltageDividerR2 = 47000.0f;   // 47kΩ (GND side)

// ADC offset correction (mV): compensates for ESP32-C6 ADC systematic error
// Adjust this value based on measured vs. actual voltage comparison
constexpr int kBatteryAdcOffsetCorrectionMv = 0;

// ============================================================================
// Signal Strength Meter Configuration
// ============================================================================

// Signal meter monitoring interval (milliseconds)
constexpr uint32_t kSignalMeterIntervalMs = 100;  // 10 readings/sec

// Peak hold time (milliseconds)
constexpr uint32_t kSignalMeterPeakHoldMs = 3000;  // 3 seconds

// S-meter calibration (AGC逆特性: 信号強→電圧低)
// 無信号時の上限電圧（この値以上は 0% とみなす）
constexpr int kSmeterAgcNoSignalVoltage = 1075;  // mV
// 強電界時の下限電圧（この値以下は 100% とクリップ）
constexpr int kSmeterAgcFullSignalVoltage = 920;  // mV

// EMAフィルタ平滑化係数 (0.0〜1.0)
// 小さいほど平滑・応答遅延増、大きいほど追従速・ノイズ多
// 推奨: 0.2（応答5サイクル≒500ms）
constexpr float kSmeterEmaAlpha = 0.2f;

// S値 (0-9) 変換定数
// signal_strength (0-100) を S値 (0-9) に変換: s = signal / kSmeterSValueDivisor
constexpr int kSmeterSValueDivisor = 11;
constexpr int kSmeterSValueMax = 9;

// S-meter パーセント基準値 (0-100%)
constexpr int kSmeterPercentMax = 100;

// Signal Meter ADC サンプリング間隔 (ms, サンプル間)
constexpr uint32_t kSignalMeterSampleIntervalMs = 1;

// Signal Meter ADC サンプル数
constexpr int kSignalMeterAdcSampleCount = 10;

// ============================================================================
// I2C Device Timing
// ============================================================================

// MCP4018 電源投入後の安定化待機時間 (ms)
constexpr uint32_t kMcp4018PowerOnDelayMs = 50;

// MCP4018 I2C リトライ前の待機時間 (ms)
constexpr uint32_t kMcp4018RetryDelayMs = 10;

// I2Cデバイス初期化後のバス安定化待機時間 (ms)
constexpr uint32_t kI2cBusStabilizeDelayMs = 100;

// メインループスリープ時間 (ms)
constexpr uint32_t kMainLoopSleepMs = 1000;

// ============================================================================
// I2C Configuration
// ============================================================================

constexpr i2c_port_t kI2cPort = I2C_NUM_0;
constexpr uint32_t kI2cFrequencyHz = 400000;

// I2C Timeout (in FreeRTOS ticks)
constexpr TickType_t kI2cTimeoutTicks = 1000 / portTICK_PERIOD_MS;

// ============================================================================
// Timing Constants
// ============================================================================

// Display update polling interval (milliseconds)
constexpr uint32_t kDisplayUpdateIntervalMs = 200;

// GPIO watch task timer interval (microseconds)
constexpr uint32_t kGpioPollingIntervalUs = 5000;  // 5ms

// GPIO watch queue timeout (FreeRTOS ticks)
constexpr TickType_t kGpioQueueTimeoutTicks = 1000 / portTICK_PERIOD_MS;

// ============================================================================
// GPIO Input Configuration
// ============================================================================

// Debounce: number of consecutive reads required for stable edge detection
constexpr int32_t kDebounceEdgeCount = 3;

// Long press timing (in polling intervals of 5ms)
constexpr int32_t kLongPressThresholdMs = 300;  // 300ms
constexpr int32_t kPollingIntervalMs = 5;       // 5ms
constexpr int32_t kLongPressThresholdCount =
    kLongPressThresholdMs / kPollingIntervalMs;  // = 60

// Long press repeat rate (in polling intervals)
constexpr int32_t kLongPressRepeatMs = 100;  // 100ms between repeats
constexpr int32_t kLongPressRepeatIntervalCount =
    kLongPressRepeatMs / kPollingIntervalMs;  // = 20

// Timer resolution for GPIO polling
constexpr uint32_t kGpioTimerResolutionUs = 1000000;  // 1 microsecond

// Message queue size for GPIO events
constexpr int32_t kGpioMessageQueueSize = 10;

// Button event queue size (UiControllerTask input)
constexpr int32_t kButtonEventQueueSize = 16;

// ============================================================================
// Volume Control Constants
// ============================================================================

// Volume input range (user-facing level)
constexpr float kVolumeInputMax = 30.0f;

// MCP4018 wiper range (0-127)
constexpr float kVolumeWiperMax = 127.0f;

// A-curve exponent for audio taper (cubic)
constexpr int kVolumeCurveExponent = 3;

// ============================================================================
// Display Formatting Constants
// ============================================================================

// Frequency display conversion factors
constexpr uint32_t kFreqToMhzDivisor = 1000000;  // Convert Hz to MHz
constexpr uint32_t kFreqToKhzModulo = 1000000;   // Extract fractional MHz
constexpr uint32_t kFreqToKhzDivisor = 10000;    // Display as hundredths

// Display line Y positions (pixels)
constexpr uint8_t kDisplayLineStatusY = 0;   // Status bar (8px, size=1)
constexpr uint8_t kDisplayLineFreqY = 8;     // Frequency (16px, size=2)
constexpr uint8_t kDisplayLineSmeterY = 24;  // S-meter (8px, size=1)

// S-meter display (left side)
constexpr int16_t kSmeterTextX = 0;  // S-value text at left edge
constexpr int16_t kSmeterBarX =
    14;  // Bar starts after "S9" text (2 chars * 6px + 2px gap)
constexpr int16_t kSmeterBarY = 25;
constexpr int16_t kSmeterBarWidth = 66;
constexpr int16_t kSmeterBarHeight = 6;
constexpr int16_t kSmeterBarInnerMax =
    64;  // Max fill width (bar_width - 2 for border)

// Volume bar dimensions (right 1/3)
constexpr int16_t kVolumeIconX = 84;
constexpr int16_t kVolumeBarX = 92;
constexpr int16_t kVolumeBarY = 25;
constexpr int16_t kVolumeBarWidth = 36;
constexpr int16_t kVolumeBarHeight = 6;
constexpr int16_t kVolumeBarInnerMax =
    34;  // Max fill width (bar_width - 2 for border)
constexpr uint8_t kVolumeMax = 30;

// Headphone icon (left side of status bar)
constexpr int16_t kHeadphoneIconX = 0;
constexpr int16_t kHeadphoneIconY = 0;
constexpr int16_t kCursorLabelX =
    11;  // Cursor label after icon (8px icon + 3px gap)

// ATT badge dimensions (left of battery icon, 2px gap)
constexpr int16_t kAttBadgeX = 93;  // Badge left edge (115 - 2 - 20)
constexpr int16_t kAttBadgeY = 0;
constexpr int16_t kAttBadgeWidth = 20;
constexpr int16_t kAttBadgeHeight = 7;

// Battery icon dimensions (top-right corner of status bar)
constexpr int16_t kBatteryIconX = 115;  // Right-aligned (128 - 13)
constexpr int16_t kBatteryIconY = 0;
constexpr int16_t kBatteryBodyWidth = 11;  // Main body width
constexpr int16_t kBatteryBodyHeight = 7;
constexpr int16_t kBatteryTerminalWidth = 2;   // Terminal nub width
constexpr int16_t kBatteryTerminalHeight = 3;  // Terminal nub height
constexpr int16_t kBatteryFillMax = 9;  // Max fill width (body - 2 for border)
constexpr float kBatteryVoltageMax = 9.0f;  // Full charge voltage
constexpr float kBatteryVoltageMin = 7.3f;  // Empty voltage

// Display contrast for dimmed state
constexpr uint8_t kDisplayContrastDimmed = 0x01;

// ============================================================================
// Data Transfer Constants
// ============================================================================

// SSD1306 I2C data transfer chunk size (bytes)
// 512 bytes = full frame buffer, sent in a single I2C transaction.
// Minimizes bus START/STOP overhead and NACK opportunities.
constexpr size_t kDisplayDataChunkSize = 512;

}  // namespace hardware_config
}  // namespace receiver_system

#endif  // HARDWARE_CONFIG_H_
