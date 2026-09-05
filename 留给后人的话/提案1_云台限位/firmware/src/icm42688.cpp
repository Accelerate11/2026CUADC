#include "icm42688.h"

#include "app_config.h"

namespace {
constexpr uint8_t kReadBit = 0x80U;
constexpr uint8_t kRegDeviceConfig = 0x11U;
constexpr uint8_t kRegAccelDataX1 = 0x1FU;
constexpr uint8_t kRegPwrMgmt0 = 0x4EU;
constexpr uint8_t kRegGyroConfig0 = 0x4FU;
constexpr uint8_t kRegAccelConfig0 = 0x50U;
constexpr uint8_t kRegWhoAmI = 0x75U;
constexpr uint8_t kRegBankSelect = 0x76U;
constexpr uint8_t kWhoAmIValue = 0x47U;

int16_t readBigEndian(const uint8_t* bytes) {
    return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8U) |
                                static_cast<uint16_t>(bytes[1]));
}
} // namespace

Icm42688::Icm42688(SPI_HandleTypeDef& spi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
    : spi_(spi), cs_port_(cs_port), cs_pin_(cs_pin) {}

void Icm42688::select() { HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET); }
void Icm42688::deselect() { HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET); }

bool Icm42688::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t bytes[2] = {static_cast<uint8_t>(reg & ~kReadBit), value};
    select();
    const HAL_StatusTypeDef result = HAL_SPI_Transmit(&spi_, bytes, 2U, 10U);
    deselect();
    return result == HAL_OK;
}

bool Icm42688::readRegisters(uint8_t reg, uint8_t* data, uint16_t length) {
    uint8_t address = static_cast<uint8_t>(reg | kReadBit);
    select();
    const HAL_StatusTypeDef tx_result = HAL_SPI_Transmit(&spi_, &address, 1U, 10U);
    const HAL_StatusTypeDef rx_result = tx_result == HAL_OK
        ? HAL_SPI_Receive(&spi_, data, length, 10U) : HAL_ERROR;
    deselect();
    return tx_result == HAL_OK && rx_result == HAL_OK;
}

uint8_t Icm42688::whoAmI() {
    uint8_t value = 0U;
    return readRegisters(kRegWhoAmI, &value, 1U) ? value : 0U;
}

bool Icm42688::begin() {
    deselect();
    HAL_Delay(5U);
    if (!writeRegister(kRegBankSelect, 0x00U) || whoAmI() != kWhoAmIValue) return false;
    if (!writeRegister(kRegDeviceConfig, 0x01U)) return false;
    HAL_Delay(2U);
    if (whoAmI() != kWhoAmIValue) return false;

    // ±500 dps/1 kHz；±4 g/1 kHz；陀螺仪和加速度计低噪声模式。
    if (!writeRegister(kRegGyroConfig0, 0x46U) ||
        !writeRegister(kRegAccelConfig0, 0x46U) ||
        !writeRegister(kRegPwrMgmt0, 0x0FU)) return false;
    HAL_Delay(50U);
    return true;
}

bool Icm42688::read(ImuSample& sample, uint32_t timestamp_us) {
    uint8_t bytes[14]{};
    if (!readRegisters(kRegAccelDataX1, bytes, sizeof(bytes))) return false;

    constexpr float kAccelScale = 8192.0F;
    constexpr float kGyroScale = 65.5F;
    sample.accel_x_g = config::kImuAccelXSign * static_cast<float>(readBigEndian(&bytes[0])) / kAccelScale;
    sample.accel_y_g = config::kImuAccelYSign * static_cast<float>(readBigEndian(&bytes[2])) / kAccelScale;
    sample.accel_z_g = config::kImuAccelZSign * static_cast<float>(readBigEndian(&bytes[4])) / kAccelScale;
    sample.gyro_x_dps = config::kImuGyroXSign * static_cast<float>(readBigEndian(&bytes[8])) / kGyroScale;
    sample.gyro_y_dps = config::kImuGyroYSign * static_cast<float>(readBigEndian(&bytes[10])) / kGyroScale;
    sample.gyro_z_dps = config::kImuGyroZSign * static_cast<float>(readBigEndian(&bytes[12])) / kGyroScale;
    sample.temperature_c = static_cast<float>(readBigEndian(&bytes[6])) / 132.48F + 25.0F;
    sample.timestamp_us = timestamp_us;
    return true;
}
