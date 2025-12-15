#pragma once
#include <driver/i2c_master.h>


#define BME280_I2C_ADDRESS_DEFAULT  ((uint8_t)0x76)    // Default sensor address
#define BME280_I2C_ADDRESS_ALTER    ((uint8_t)0x77)    // Alternative sensor address

#define BME280_CHIP_ID_REG          ((uint8_t)0xD0)    // Chip ID register
#define BME280_CHIP_ID_VAL          ((uint8_t)0x60)    // Default ID value

#define BME280_RESET_REG            ((uint8_t)0xE0)

