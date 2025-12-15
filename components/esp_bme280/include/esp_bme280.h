#pragma once
#include <driver/i2c_master.h>


#define BME280_I2C_ADDRESS_DEFAULT  ((uint8_t)0x76)    // Default sensor address
#define BME280_I2C_ADDRESS_ALTER    ((uint8_t)0x77)    // Alternative sensor address

#define BME280_CHIP_ID_REG          ((uint8_t)0xD0)    // Chip ID register
#define BME280_CHIP_ID_VAL          ((uint8_t)0x60)    // Default ID value

#define BME280_RESET_REG            ((uint8_t)0xE0)    // Soft reset register
#define BME280_STATUS_REG           ((uint8_t)0xF3)    // Measurement/result status register

#define BME280_CTRL_HUM_REG         ((uint8_t)0xF2)    // Humidity oversampling control
#define BME280_CTRL_MEAS_REG        ((uint8_t)0xF4)    // Temperature and pressure oversampling control + mode field
#define BME280_CONFIG_REG           ((uint8_t)0xF5)    // Standby time, filter setting, SPI enable flag


#define BME280_TEMPERATURE_REG        ((uint8_t)0xFA)
#define BME280_TEMPERATURE_REG_COUNT  3

#define BME280_PRESSURE_REG           ((uint8_t)0xF7)
#define BME280_PRESSURE_REG_COUNT     3

#define BME280_HUMIDITY_RED           ((uint8_t)0xFD)
#define BME280_HUMIDITY_REG_COUNT     2


typedef enum : uint8_t {
    BME280_OVERSAMPLING_SKIP,
    BME280_OVERSAMPLING_X1,
    BME280_OVERSAMPLING_X2,
    BME280_OVERSAMPLING_X4,
    BME280_OVERSAMPLING_X8,
    BME280_OVERSAMPLING_X16,
} bme280_oversampling_t;

typedef enum : uint8_t {
    BME280_MODE_SLEEP,
    BME280_MODE_FORCED,
    BME280_MODE_FORCED_ALT,
    BME280_MODE_NORMAL,
} bme280_mode_t;

typedef enum : uint8_t {
    BME280_FILTER_OFF,
    BME280_FILTER_X2,
    BME280_FILTER_X4,
    BME280_FILTER_X8,
    BME280_FILTER_X16,
} bme280_filter_t;

typedef enum : uint8_t {
    BME280_STANDBY_MS_0_5 = 0b000,
    BME280_STANDBY_MS_10 = 0b110,
    BME280_STANDBY_MS_20 = 0b111,
    BME280_STANDBY_MS_62_5 = 0b001,
    BME280_STANDBY_MS_125 = 0b010,
    BME280_STANDBY_MS_250 = 0b011,
    BME280_STANDBY_MS_500 = 0b100,
    BME280_STANDBY_MS_1000 = 0b101
} bme280_standby_duration_t;

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
} bme280_calib_t;

typedef struct {
    uint8_t padding_up : 4;
    uint8_t measuring : 1;
    uint8_t padding_mid : 2;
    uint8_t im_update : 1;
} bme280_status_t;

typedef struct {
    uint8_t padding : 5;
    bme280_oversampling_t osrs_h : 3;
} bme280_ctrl_hum_t;

typedef struct {
    bme280_oversampling_t osrs_t : 3;
    bme280_oversampling_t osrs_p : 3;
    bme280_mode_t mode : 2;
} bme280_ctrl_meas_t;

typedef struct {
    bme280_standby_duration_t t_sb : 3;
    bme280_filter_t filter : 3;
    uint8_t unused : 1;
    uint8_t spi3w_en : 1;
} bme280_config_t;