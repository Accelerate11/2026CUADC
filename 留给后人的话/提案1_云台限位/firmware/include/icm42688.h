#pragma once

#include "stm32g4xx_hal.h"
#include "types.h"

class Icm42688 {
public:
    Icm42688(SPI_HandleTypeDef& spi, GPIO_TypeDef* cs_port, uint16_t cs_pin);
    bool begin();
    bool read(ImuSample& sample, uint32_t timestamp_us);
    uint8_t whoAmI();

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* data, uint16_t length);
    void select();
    void deselect();

    SPI_HandleTypeDef& spi_;
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;
};
