#include "adxl343.h"

ADXL343::ADXL343(TwoWire &wirePort, uint8_t address) : wire(wirePort), i2cAddress(address) {
}

bool ADXL343::begin() {
    // Initialize I2C communication
    wire.begin();

    // Check device ID
    uint8_t deviceId = readRegister8(REG_DEVID);
    if (deviceId != 0xE5) {
        Log.error("ADXL343 not found. Device ID: 0x%02X", deviceId);
        return false;
    } else {
        Log.info("ADXL343 detected. Device ID: 0x%02X", deviceId);
    }

    // Set data format to +/-2g, full resolution
    writeRegister8(REG_DATA_FORMAT, 0x08);

    // Enable measurements
    writeRegister8(REG_POWER_CTL, 0x08);

    Log.info("ADXL343 initialization complete.");
    return true;
}

void ADXL343::readAcceleration(int16_t *x, int16_t *y, int16_t *z) {
    wire.beginTransmission(i2cAddress);
    wire.write(REG_DATAX0);
    wire.endTransmission(false);
    wire.requestFrom(i2cAddress, (uint8_t)6);

    if (wire.available() >= 6) {
        uint8_t x0 = wire.read();
        uint8_t x1 = wire.read();
        uint8_t y0 = wire.read();
        uint8_t y1 = wire.read();
        uint8_t z0 = wire.read();
        uint8_t z1 = wire.read();

        // Combine MSB and LSB
        *x = (int16_t)((x1 << 8) | x0);
        *y = (int16_t)((y1 << 8) | y0);
        *z = (int16_t)((z1 << 8) | z0);
    } else {
        Log.warn("Insufficient data available from ADXL343.");
    }
}

void ADXL343::readAccelerationG(float *x, float *y, float *z) {
    int16_t xi, yi, zi;
    readAcceleration(&xi, &yi, &zi);

    // Assuming default settings: full resolution mode with +/-2g range
    // Sensitivity is 256 LSB/g in full resolution mode at +/-2g
    // So scale factor is 1/256 = 0.00390625 g/LSB

    float scaleFactor = 0.00390625f;
    *x = xi * scaleFactor;
    *y = yi * scaleFactor;
    *z = zi * scaleFactor;
}

uint8_t ADXL343::readRegister8(uint8_t reg) {
    wire.beginTransmission(i2cAddress);
    wire.write(reg);
    wire.endTransmission(false); // Restart for read
    wire.requestFrom(i2cAddress, (uint8_t)1);
    if (wire.available()) {
        return wire.read();
    }
    return 0;
}

void ADXL343::writeRegister8(uint8_t reg, uint8_t value) {
    wire.beginTransmission(i2cAddress);
    wire.write(reg);
    wire.write(value);
    wire.endTransmission();
}
