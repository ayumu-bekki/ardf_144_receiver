#include "mcp4018.h"

#include <memory>

#include "i2c_util.h"
#include "logger.h"
#include "util.h"

namespace receiver_system {

MCP4018::MCP4018()
    : dev_handle_(nullptr) {}

MCP4018::~MCP4018() {
  if (dev_handle_) {
    i2c_master_bus_rm_device(dev_handle_);
    dev_handle_ = nullptr;
  }
}

void MCP4018::Setup() {
  // Get the I2C bus handle
  i2c_master_bus_handle_t bus_handle = i2c_util::GetBusHandle();
  if (!bus_handle) {
    ESP_LOGE(kTag, "I2C bus not initialized");
    return;
  }

  // Add device to the bus
  i2c_device_config_t dev_config = {};
  dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_config.device_address = kI2cDefaultAddr;
  dev_config.scl_speed_hz = i2c_util::kI2cMasterFrequencyHz;

  esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "Failed to add MCP4018 device: %s", esp_err_to_name(ret));
    return;
  }

  // データシート仕様: 電源投入後20ms待機が必要
  util::SleepMillisecond(50);

  ESP_LOGI(kTag, "MCP4018 initialized at address 0x%02X", kI2cDefaultAddr);
}

void MCP4018::SetWiper(uint8_t value) {
  // 値の範囲チェック
  if (value > kWiperMax) {
    ESP_LOGW(kTag, "Wiper value %u exceeds max %u, clamping to max", value, kWiperMax);
    value = kWiperMax;
  }

  ESP_LOGD(kTag, "Setting wiper to %u (0=max resistance, 127=min resistance)", value);

  // MCP4018へ1バイト書き込み
  Write(value);
}

void MCP4018::Write(const uint8_t value) {
  if (!dev_handle_) {
    ESP_LOGE(kTag, "Device handle not initialized");
    return;
  }

  // データ書き込み (上位1ビットは無視されるため、0-127の範囲で有効)
  uint8_t write_buf = value & 0x7F;

  esp_err_t ret = i2c_master_transmit(dev_handle_, &write_buf, 1, i2c_util::kI2cTimeoutMs);
  if (ret != ESP_OK) {
    // バス安定待機後に1回だけリトライ
    util::SleepMillisecond(10);
    ret = i2c_master_transmit(dev_handle_, &write_buf, 1, i2c_util::kI2cTimeoutMs);
    if (ret != ESP_OK) {
      ESP_LOGE(kTag, "MCP4018 I2C write failed: %s", esp_err_to_name(ret));
    }
  }
}

}  // namespace receiver_system
