/**
 * @file esp_bme280.h
 * @brief ESP-IDF driver for Bosch BME280 temperature, pressure and humidity sensor
 *
 * Copyright (c) 2025 Mikołaj Kraszewski
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Mikołaj Kraszewski (@infinite-dark)
 *
 * This header provides definitions, enums, structs, and function prototypes
 * for interfacing with the BME280 sensor over I2C. The BME280 measures
 * temperature, pressure, and humidity. It supports oversampling, filtering,
 * and various operating modes (sleep, forced, normal).
 *
 * Key features:
 * - I2C communication via ESP-IDF's I2C master driver.
 * - Soft reset and chip ID verification.
 * - Calibration data handling for accurate measurements.
 * - Configurable oversampling, standby duration, and IIR filtering.
 * - Burst read support for calibration registers (auto-increment).
 *
 * Usage overview:
 * 1. Create a sensor handle with bme280_create() or bme280_create_default().
 * 2. Initialize with bme280_init() or bme280_init_default().
 * 3. Read measurements using bme280_measure().
 * 4. Delete the handle with bme280_delete() when done.
 *
 * @note This driver assumes the sensor is connected via I2C. SPI is not supported here.
 * @note Register map is partially defined to shorten the code by leveraging BME280's auto-increment for burst reads.
 * @note For full, accurate information on the sensor's register map, please refer to Bosch's official documentation.
 * @note All functions return ESP_ERR_* codes; check for ESP_OK on success.
 */

#pragma once
#include <stdint.h>
#include <driver/i2c_master.h>

/* ============================================================================
 * I2C Addresses and Core Registers
 * ============================================================================ */

/**
 * @brief Default I2C address for BME280 (SDO pin low).
 */
#define BME280_I2C_ADDRESS_DEFAULT  ((uint8_t)0x76)

/**
 * @brief Alternative I2C address for BME280 (SDO pin high).
 */
#define BME280_I2C_ADDRESS_ALTER    ((uint8_t)0x77)

/**
 * @brief Chip ID register address.
 */
#define BME280_CHIP_ID_REG          ((uint8_t)0xD0)

/**
 * @brief Expected Chip ID value for BME280.
 */
#define BME280_CHIP_ID_VAL          ((uint8_t)0x60)

/**
 * @brief Soft reset register address.
 */
#define BME280_RESET_REG            ((uint8_t)0xE0)

/**
 * @brief Magic value to trigger soft reset.
 */
#define BME280_RESET_WORD           ((uint8_t)0xB6)

/**
 * @brief Status register for measurement and update flags.
 */
#define BME280_STATUS_REG           ((uint8_t)0xF3)

/**
 * @brief Humidity oversampling control register.
 */
#define BME280_CTRL_HUM_REG         ((uint8_t)0xF2)

/**
 * @brief Temperature/pressure oversampling and mode control register.
 */
#define BME280_CTRL_MEAS_REG        ((uint8_t)0xF4)

/**
 * @brief Configuration register (standby, filter, SPI mode).
 */
#define BME280_CONFIG_REG           ((uint8_t)0xF5)

/* ============================================================================
 * Calibration Data Registers
 * ============================================================================ */

/**
 * @brief Starting register for temperature and pressure calibration data.
 */
#define BME280_CALIB_TEMP_PRES_FIRST_REG    ((uint8_t)0x88)

/**
 * @brief Ending register for temperature and pressure calibration data.
 */
#define BME280_CALIB_TEMP_PRES_LAST_REG     ((uint8_t)0x9F)

/**
 * @brief Number of bytes for temperature and pressure calibration (24).
 */
#define BME280_CALIB_TEMP_PRESS_REG_COUNT   ((uint8_t)(BME280_CALIB_TEMP_PRES_LAST_REG - BME280_CALIB_TEMP_PRES_FIRST_REG + 1))

/**
 * @brief Starting register for humidity calibration data (first part).
 */
#define BME280_CALIB_HUMIDITY_FIRST_REG     ((uint8_t)0xA1)

/**
 * @brief Starting register for humidity calibration data (second part, non-continuous).
 */
#define BME280_CALIB_HUMIDITY_SECOND_REG    ((uint8_t)0xE1)

/**
 * @brief Ending register for humidity calibration data.
 */
#define BME280_CALIB_HUMIDITY_LAST_REG      ((uint8_t)0xE7)

/**
 * @brief Total byte count for humidity calibration data (8 bytes, non-continuous read).
 */
#define BME280_CALIB_HUMIDITY_REG_COUNT     8

/* ============================================================================
 * Raw Measurement Registers
 * ============================================================================ */

/**
 * @brief Starting register for raw temperature data (MSB first).
 */
#define BME280_TEMPERATURE_REG        ((uint8_t)0xFA)

/**
 * @brief Byte size of raw temperature value (20-bit, 3 bytes).
 */
#define BME280_TEMPERATURE_SIZE       3

/**
 * @brief Starting register for raw pressure data (MSB first).
 */
#define BME280_PRESSURE_REG           ((uint8_t)0xF7)

/**
 * @brief Byte size of raw pressure value (20-bit, 3 bytes).
 */
#define BME280_PRESSURE_SIZE          3

/**
 * @brief Starting register for raw humidity data (MSB first).
 */
#define BME280_HUMIDITY_REG           ((uint8_t)0xFD)

/**
 * @brief Byte size of raw humidity value (16-bit, 2 bytes).
 */
#define BME280_HUMIDITY_SIZE          2

/* ============================================================================
 * Enums for Configuration Options
 * ============================================================================ */

/**
 * @brief Oversampling rates for sensor measurements.
 *
 * Higher rates improve accuracy but increase power consumption and measurement time.
 * SKIP means the sensor is disabled for that parameter.
 */
typedef enum : uint8_t {
    BME280_OVERSAMPLING_SKIP,   ///< Skip measurement (disabled).
    BME280_OVERSAMPLING_X1,     ///< 1x oversampling.
    BME280_OVERSAMPLING_X2,     ///< 2x oversampling.
    BME280_OVERSAMPLING_X4,     ///< 4x oversampling.
    BME280_OVERSAMPLING_X8,     ///< 8x oversampling.
    BME280_OVERSAMPLING_X16,    ///< 16x oversampling.
} bme280_oversampling_t;

/**
 * @brief Operating modes of the BME280.
 */
typedef enum : uint8_t {
    BME280_MODE_SLEEP,      ///< Sleep mode: All sensors off, minimal power.
    BME280_MODE_FORCED,     ///< Forced mode: Single measurement, then sleep.
    BME280_MODE_FORCED_ALT, ///< Exactly the same as forced mode. Just for coding convenience.
    BME280_MODE_NORMAL,     ///< Normal mode: Continuous measurements with standby periods.
} bme280_mode_t;

/**
 * @brief IIR filter coefficients for reducing noise.
 *
 * Higher coefficients average more samples, reducing bandwidth.
 */
typedef enum : uint8_t {
    BME280_FILTER_OFF,  ///< No filtering.
    BME280_FILTER_X2,   ///< 2x averaging.
    BME280_FILTER_X4,   ///< 4x averaging.
    BME280_FILTER_X8,   ///< 8x averaging.
    BME280_FILTER_X16,  ///< 16x averaging.
} bme280_filter_t;

/**
 * @brief Standby duration between measurements in normal mode (in ms).
 */
typedef enum : uint8_t {
    BME280_STANDBY_MS_0_5 = 0b000,    ///< 0.5 ms.
    BME280_STANDBY_MS_62_5 = 0b001,   ///< 62.5 ms.
    BME280_STANDBY_MS_125 = 0b010,    ///< 125 ms.
    BME280_STANDBY_MS_250 = 0b011,    ///< 250 ms.
    BME280_STANDBY_MS_500 = 0b100,    ///< 500 ms.
    BME280_STANDBY_MS_1000 = 0b101,   ///< 1000 ms.
    BME280_STANDBY_MS_10 = 0b110,     ///< 10 ms.
    BME280_STANDBY_MS_20 = 0b111,     ///< 20 ms.
} bme280_standby_duration_t;

/* ============================================================================
 * Calibration Data Structures
 * ============================================================================ */

/**
 * @brief Union for temperature and pressure calibration coefficients.
 *
 * dig_T1 to dig_T3: Temperature coefficients.
 * dig_P1 to dig_P9: Pressure coefficients.
 *
 * @note Stored as raw bytes for burst read from 0x88-0x9F.
 * @note Automatic burst-read correctly populates the fields on little-endian systems.
 */
typedef union {
    struct {
        uint16_t dig_T1;    ///< Temperature calibration coefficient T1 (unsigned).
        int16_t  dig_T2;    ///< Temperature calibration coefficient T2 (signed).
        int16_t  dig_T3;    ///< Temperature calibration coefficient T3 (signed).
        uint16_t dig_P1;    ///< Pressure calibration coefficient P1 (unsigned).
        int16_t  dig_P2;    ///< Pressure calibration coefficient P2 (signed).
        int16_t  dig_P3;    ///< Pressure calibration coefficient P3 (signed).
        int16_t  dig_P4;    ///< Pressure calibration coefficient P4 (signed).
        int16_t  dig_P5;    ///< Pressure calibration coefficient P5 (signed).
        int16_t  dig_P6;    ///< Pressure calibration coefficient P6 (signed).
        int16_t  dig_P7;    ///< Pressure calibration coefficient P7 (signed).
        int16_t  dig_P8;    ///< Pressure calibration coefficient P8 (signed).
        int16_t  dig_P9;    ///< Pressure calibration coefficient P9 (signed).
    } __attribute__((packed)) calib;    ///< Parsed calibration values.
    uint8_t raw[BME280_CALIB_TEMP_PRESS_REG_COUNT]; ///< Raw register bytes.
} bme280_calib_temp_press_t;

static_assert(sizeof(bme280_calib_temp_press_t) == BME280_CALIB_TEMP_PRESS_REG_COUNT, "bme280_calib_temp_press_t must be exactly 24 bytes");

/**
 * @brief Struct for humidity calibration coefficients.
 *
 * Read from non-continuous registers: 0xA1 (H1), 0xE1-E7 (H2-H6).
 *
 * @note dig_H4 and dig_H5 require bit manipulation during parsing.
 */
typedef struct {
    uint8_t  dig_H1;    ///< Humidity calibration coefficient H1 (unsigned).
    int16_t  dig_H2;    ///< Humidity calibration coefficient H2 (signed).
    uint8_t  dig_H3;    ///< Humidity calibration coefficient H3 (unsigned).
    int16_t  dig_H4;    ///< Humidity calibration coefficient H4 (signed, bits 11-4 in byte 0xE4, bits 3-0 in 0xE3[7:4]).
    int16_t  dig_H5;    ///< Humidity calibration coefficient H5 (signed, bits 11-0 in byte 0xE5 and 0xE6).
    int8_t   dig_H6;    ///< Humidity calibration coefficient H6 (signed).
} __attribute__((packed)) bme280_calib_humidity_t;

// Note: Actual read is 8 bytes; H1 separate.
static_assert(sizeof(bme280_calib_humidity_t) == 9, "bme280_calib_humidity_t must be exactly 9 bytes");

/* ============================================================================
 * Register Bitfield Structures
 * ============================================================================ */

/**
 * @brief Bitfield for status register (0xF3).
 */
typedef struct {
    uint8_t im_update : 1;      ///< 1 if calibration/update in progress.
    uint8_t padding_middle : 2; ///< Reserved (0).
    uint8_t measuring : 1;      ///< 1 if measurement in progress.
    uint8_t padding_upper : 4;  ///< Reserved (0).
} __attribute__((packed)) bme280_status_t;

static_assert(sizeof(bme280_status_t) == 1, "bme280_status_t must be exactly 1 byte");

/**
 * @brief Bitfield for humidity control register (0xF2).
 */
typedef struct {
    bme280_oversampling_t osrs_h : 3;   ///< Humidity oversampling rate.
    uint8_t padding : 5;                ///< Reserved (0).
} __attribute__((packed)) bme280_ctrl_hum_t;

static_assert(sizeof(bme280_ctrl_hum_t) == 1, "bme280_ctrl_hum_t must be exactly 1 byte");

/**
 * @brief Bitfield for measurement control register (0xF4).
 */
typedef struct {
    bme280_mode_t mode : 2;             ///< Operating mode.
    bme280_oversampling_t osrs_p : 3;   ///< Pressure oversampling rate.
    bme280_oversampling_t osrs_t : 3;   ///< Temperature oversampling rate.
} __attribute__((packed)) bme280_ctrl_meas_t;

static_assert(sizeof(bme280_ctrl_meas_t) == 1, "bme280_ctrl_meas_t must be exactly 1 byte");

/**
 * @brief Bitfield for configuration register (0xF5).
 */
typedef struct {
    uint8_t spi3w_en : 1;                   ///< 1 to enable 3-wire SPI (I2C unaffected).
    uint8_t unused : 1;                     ///< Reserved (0).
    bme280_filter_t filter : 3;             ///< IIR filter coefficient.
    bme280_standby_duration_t t_sb : 3;     ///< Standby time in normal mode.
} __attribute__((packed)) bme280_config_t;

static_assert(sizeof(bme280_config_t) == 1, "bme280_config_t must be exactly 1 byte");

/** ============================================================================
 * IMPORTANT: Bit-field layout assumptions
 * ============================================================================
 *
 * All bitfield structs in this driver (bme280_status_t, bme280_ctrl_hum_t,
 * bme280_ctrl_meas_t, bme280_config_t) assume the following:
 *
 * 1. Bit-fields are filled from least-significant to most-significant bit (LSB → MSB)
 * within a storage unit. This is the common behavior for little-endian ABIs.
 *
 * 2. The compiler lays out bit-fields in **declaration order** (the first declared
 * field is assigned to the lowest bit offsets).
 *
 * The C standard treats bit-field layout as implementation-defined, and therefore
 * it is **not portable** to all compilers.
 *
 * This driver is intended for GCC-based toolchains (ESP-IDF default).
 * If porting to a different compiler, you must verify the bit-field packing behavior
 * or replace these structs with manual bit masking and shifting to ensure portability.
 */

/* ============================================================================
 * High-Level Configuration and Measurement Structs
 * ============================================================================ */

/**
 * @brief Device configuration for BME280 initialization.
 */
typedef struct {
    bme280_mode_t measurement_mode;         ///< Operating mode (e.g., BME280_MODE_NORMAL).
    bme280_oversampling_t temperature_oversampling; ///< Temp oversampling (e.g., BME280_OVERSAMPLING_X1).
    bme280_oversampling_t pressure_oversampling;    ///< Pressure oversampling.
    bme280_oversampling_t humidity_oversampling;    ///< Humidity oversampling.
    bme280_filter_t filter_mode;                    ///< IIR filter setting.
    bme280_standby_duration_t standby_duration;     ///< Standby time in normal mode.
} bme280_device_config_t;

/**
 * @brief Processed measurement output (in physical units).
 */
typedef struct {
    double temperature;     ///< Temperature in degrees Celsius.
    double pressure;        ///< Pressure in hectopascals (hPa).
    double humidity;        ///< Relative humidity in percent (%RH).
} bme280_measurement_t;

/* ============================================================================
 * Opaque Handle and Typedefs
 * ============================================================================ */

/**
 * @brief Opaque struct for internal sensor state.
 */
typedef struct bme280_sensor bme280_sensor_t;

/**
 * @brief Handle to a BME280 sensor instance.
 */
typedef bme280_sensor_t * bme280_handle_t;

/**
 * @brief Const handle for read-only access.
 */
typedef const bme280_sensor_t * const bme280_const_handle_t;

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/**
 * @brief Create a BME280 sensor handle on the given I2C bus.
 *
 * Performs chip ID check and soft reset.
 *
 * @param[in]  bus_handle     I2C master bus handle from ESP-IDF.
 * @param[in]  dev_cfg        I2C device config (address, etc.).
 * @param[out] out_handle     Pointer to store the created handle.
 * @return ESP_OK on success, or ESP_ERR_* on failure (e.g., wrong chip ID).
 */
esp_err_t bme280_create(i2c_master_bus_handle_t bus_handle, const i2c_device_config_t * dev_cfg, bme280_handle_t *out_handle);

/**
 * @brief Create a default BME280 sensor handle (address 0x76).
 *
 * Convenience wrapper for bme280_create() with default device config.
 *
 * @param[in]  bus_handle     I2C master bus handle.
 * @param[in]  dev_addr       I2C device address (default or alternative).
 * @param[out] out_handle     Pointer to store the created handle.
 * @return ESP_OK on success, or ESP_ERR_* on failure.
 */
esp_err_t bme280_create_default(i2c_master_bus_handle_t bus_handle, uint8_t dev_addr, bme280_handle_t *out_handle);

/**
 * @brief Initialize the sensor with custom configuration.
 *
 * Writes config to CTRL_HUM, CTRL_MEAS, and CONFIG registers.
 * Reads and stores calibration data.
 *
 * @param[in,out] bme280_sensor     Sensor handle.
 * @param[in]     bme280_device_config  Configuration parameters.
 * @return ESP_OK on success, or ESP_ERR_* on I2C/comms failure.
 */
esp_err_t bme280_init(bme280_handle_t bme280_sensor, const bme280_device_config_t * bme280_device_config);

/**
 * @brief Initialize with default settings (1x oversampling, normal mode, 125ms standby, no filter).
 *
 * Convenience wrapper for bme280_init().
 *
 * @param[in,out] bme280_sensor     Sensor handle.
 * @return ESP_OK on success, or ESP_ERR_* on failure.
 */
esp_err_t bme280_init_default(bme280_handle_t bme280_sensor);

/**
 * @brief Delete the sensor handle and free resources.
 *
 * Puts sensor into sleep mode before cleanup.
 *
 * @param[in,out] bme280_sensor     Pointer to handle; set to NULL on success.
 * @return ESP_OK on success, or ESP_ERR_* if invalid handle.
 */
esp_err_t bme280_delete(bme280_handle_t * bme280_sensor);

/**
 * @brief Perform a measurement and return processed values.
 *
 * Triggers a forced measurement if needed, waits for completion,
 * reads raw data, applies calibration, and compensates to physical units.
 *
 * @param[in]  bme280_sensor     Const sensor handle.
 * @param[out] output_handle     Pointer to store measurement results.
 * @return ESP_OK on success, or ESP_ERR_* on timeout/comms failure.
 */
esp_err_t bme280_measure(bme280_const_handle_t bme280_sensor, bme280_measurement_t * output_handle);
