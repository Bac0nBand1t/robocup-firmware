#pragma once

#include "mtrain.hpp"
#include "SPI.hpp"
#include <memory>

class ICM20948 {
public:
    ICM20948(std::shared_ptr<SPI> imuSPI, PinName cs_pin);

    bool initialize();

    float gyro_z();

private:
    void write_register(uint8_t bank, uint8_t address, uint8_t value);

    uint8_t read_register(uint8_t bank, uint8_t address);
    
    void chip_select(bool cs_state);

    std::shared_ptr<SPI> imuSPI;

    DigitalOut nCs;
};
