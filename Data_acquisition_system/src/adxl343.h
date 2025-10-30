#ifndef ADXL343_H
#define ADXL343_H

#include "Particle.h"

// ADXL343 I2C address
#define ADXL343_ADDRESS 0x53

// ADXL343 Registers
#define REG_DEVID          0x00
#define REG_POWER_CTL      0x2D
#define REG_DATA_FORMAT    0x31
#define REG_DATAX0         0x32

class ADXL343 {
public:
    ADXL343(TwoWire &wirePort = Wire, uint8_t address = ADXL343_ADDRESS);

    bool begin();
    void readAcceleration(int16_t *x, int16_t *y, int16_t *z);
    void readAccelerationG(float *x, float *y, float *z);

private:
    uint8_t readRegister8(uint8_t reg);
    void writeRegister8(uint8_t reg, uint8_t value);

    TwoWire &wire;
    uint8_t i2cAddress;
};

#endif // ADXL343_H
