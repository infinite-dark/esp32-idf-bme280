/**
 * @file esp_bme280.c
 * @brief ESP-IDF driver implementation for Bosch BME280 environmental sensor.
 *
 * This file contains the full implementation of the BME280 driver, including:
 * - Sensor handle creation and cleanup
 * - Soft reset and chip ID verification
 * - Loading of factory calibration coefficients
 * - Configuration of oversampling, filter, standby time, and mode
 * - Measurement triggering (forced mode) and reading
 * - Temperature, pressure, and humidity compensation formulas (official Bosch implementation)
 *
 * The driver uses the ESP-IDF I2C master driver and FreeRTOS for timing.
 * All public functions return ESP_OK on success or an ESP_ERR_* code on failure.
 *
 * @note This implementation follows Bosch's official compensation formulas exactly.
 * @note Bitfield structs assume GCC bitfield layout (lsb → msb, declaration order).
 * @note Designed for I2C only (no SPI support).
 *
 * Copyright (c) 2025 Mikołaj Kraszewski
 * SPDX-License-Identifier: Apache-2.0
 *
 * @see esp_bme280.h for public API and detailed documentation
 */

#include <stdlib.h>
#include "esp_bme280.h"
#include <esp_check.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define BME280_TIMEOUT_MS_DEFAULT       100     ///< Default timeout for I2C operations and measurement wait
#define BME280_I2C_FREQUENCY_DEFAULT    400000  ///< Default I2C clock speed (400 kHz)

static const char* const API_TAG = "bme280_api";

/**
 * @brief Internal opaque sensor context structure.
 */
struct bme280_sensor {
    i2c_master_dev_handle_t     dev_handle;     ///< I2C device handle from ESP-IDF I2C master
    bme280_ctrl_meas_t          ctrl_meas;      ///< Cached measurement control register value
    bme280_ctrl_hum_t           ctrl_hum;       ///< Cached humidity control register value
    bme280_config_t             config;         ///< Cached configuration register value
    bme280_calib_temp_press_t   tp_calib;       ///< Temperature & pressure calibration coefficients
    bme280_calib_humidity_t     h_calib;        ///< Humidity calibration coefficients
};

/* -------------------------------------------------------------------------
 * Internal helper function prototypes
 * ------------------------------------------------------------------------- */

static inline esp_err_t bme280_read_reg(bme280_const_handle_t bme280_sensor, uint8_t reg, uint8_t *resp);
static inline esp_err_t bme280_read_reg_many(bme280_const_handle_t bme280_sensor, uint8_t start_reg, size_t count, uint8_t *resp);
static inline esp_err_t bme280_write_reg_raw(bme280_const_handle_t bme280_sensor, uint8_t reg, uint8_t value);
static inline esp_err_t bme280_write_reg_field(bme280_const_handle_t bme280_sensor, uint8_t reg_addr, const void *field_ptr);
static inline esp_err_t bme280_wait_sensor_ready(bme280_const_handle_t bme280_sensor, TickType_t timeout_ticks);

static inline esp_err_t bme280_assign_config_params(bme280_handle_t bme280_sensor, const bme280_device_config_t *bme280_device_config);
static inline esp_err_t bme280_load_calibration_data(bme280_handle_t bme280_sensor);

/* -------------------------------------------------------------------------
 * Compensation functions (Bosch reference implementation)
 * ------------------------------------------------------------------------- */

int32_t compensate_temperature(bme280_const_handle_t bme280_sensor, int32_t adc_T, int32_t *T_fine);
uint32_t compensate_pressure(bme280_const_handle_t bme280_sensor, int32_t adc_P, int32_t T_fine);
uint32_t compensate_humidity(bme280_const_handle_t bme280_sensor, int32_t adc_H, int32_t T_fine);

/* -------------------------------------------------------------------------
 * Public API Implementation
 * ------------------------------------------------------------------------- */

/**
 * @brief Create and initialize a new BME280 sensor instance.
 *
 * Probes the I2C bus to confirm device presence, adds the device to the bus,
 * and allocates the internal context.
 *
 * @param[in] bus_handle I2C master bus handle
 * @param[in] dev_cfg I2C device configuration (address, speed, etc.)
 * @param[out] out_handle Pointer to store the created sensor handle
 * @return ESP_OK on success, ESP_ERR_* otherwise
 */
esp_err_t bme280_create(i2c_master_bus_handle_t bus_handle, const i2c_device_config_t * const dev_cfg, bme280_handle_t * const out_handle) {

    ESP_LOGI(API_TAG, "Attempting to create BME280 sensor context...");

    ESP_RETURN_ON_FALSE(bus_handle && dev_cfg && out_handle, ESP_ERR_INVALID_ARG, API_TAG, "Invalid arguments");

    esp_err_t ret;

    // Probe to make sure device responds
    ret = i2c_master_probe(bus_handle, dev_cfg->device_address, BME280_TIMEOUT_MS_DEFAULT);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to find the BME280 at address 0x%x", dev_cfg->device_address);

    i2c_master_dev_handle_t dev_handle;
    ret = i2c_master_bus_add_device(bus_handle, dev_cfg, &dev_handle);
    ESP_GOTO_ON_ERROR(ret, err_return, API_TAG, "Failed to add the device to the I2C bus");

    struct bme280_sensor *sensor = calloc(1, sizeof(struct bme280_sensor));
    ESP_GOTO_ON_FALSE(sensor, ESP_ERR_NO_MEM, err_release, API_TAG, "Failed to allocate the BME280 sensor context");

    sensor->dev_handle = dev_handle;
    *out_handle = sensor;

    ESP_LOGI(API_TAG, "Success creating BME280 context");
    return ESP_OK;

err_release:
    ret = i2c_master_bus_rm_device(dev_handle);
    if (ret != ESP_OK) ESP_LOGE(API_TAG, "Failed to remove I2C device");
err_return:
    ESP_LOGE(API_TAG, "Failed to create BME280 context");
    return ESP_FAIL;

}

/**
 * @brief Convenience wrapper: create sensor using default or alternative address.
 */
esp_err_t bme280_create_default(i2c_master_bus_handle_t bus_handle, const uint8_t dev_addr, bme280_handle_t * const out_handle) {

    if (dev_addr != BME280_I2C_ADDRESS_DEFAULT && dev_addr != BME280_I2C_ADDRESS_ALTER) {
        ESP_LOGW(API_TAG, "Uncommon BME280 I2C address 0x%02x", dev_addr);
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = BME280_I2C_FREQUENCY_DEFAULT,
        .scl_wait_us = 0,
        .flags = { .disable_ack_check = false }
    };

    return bme280_create(bus_handle, &dev_cfg, out_handle);

}

/**
 * @brief Initialize the sensor: reset, load calibration, apply user configuration.
 *
 * Sequence:
 * 1. Verify chip ID
 * 2. Perform soft reset
 * 3. Wait for sensor to become ready
 * 4. Load factory calibration coefficients
 * 5. Apply user configuration (oversampling, mode, filter, standby)
 */
esp_err_t bme280_init(bme280_handle_t bme280_sensor, const bme280_device_config_t * const bme280_device_config) {

    ESP_RETURN_ON_FALSE(bme280_sensor && bme280_device_config, ESP_ERR_INVALID_ARG, API_TAG, "Invalid arguments");

    ESP_LOGI(API_TAG, "Initializing the BME280 sensor...");

    esp_err_t ret;
    uint8_t chip_id;

    ret = bme280_read_reg(bme280_sensor, BME280_CHIP_ID_REG, &chip_id);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to read chip ID");
    ESP_RETURN_ON_FALSE(chip_id == BME280_CHIP_ID_VAL, ESP_ERR_INVALID_RESPONSE, API_TAG, "Invalid chip ID (expected 0x60, got 0x%02x)", chip_id);

    // Soft reset
    ret = bme280_write_reg_raw(bme280_sensor, BME280_RESET_REG, BME280_RESET_WORD);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to perform soft reset");

    // Wait for reset & NVM copy to complete
    ret = bme280_wait_sensor_ready(bme280_sensor, pdMS_TO_TICKS(BME280_TIMEOUT_MS_DEFAULT));
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Sensor failed to become ready after reset");

    ret = bme280_load_calibration_data(bme280_sensor);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to load calibration data");

    ret = bme280_assign_config_params(bme280_sensor, bme280_device_config);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to assign configuration parameters");

    // Write configuration registers (order matters: config → ctrl_hum → ctrl_meas)
    ret = bme280_write_reg_field(bme280_sensor, BME280_CONFIG_REG, &bme280_sensor->config);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to write CONFIG register");

    ret = bme280_write_reg_field(bme280_sensor, BME280_CTRL_HUM_REG, &bme280_sensor->ctrl_hum);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to write CTRL_HUM register");

    ret = bme280_write_reg_field(bme280_sensor, BME280_CTRL_MEAS_REG, &bme280_sensor->ctrl_meas);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to write CTRL_MEAS register");

    ESP_LOGI(API_TAG, "Success initializing the BME280 sensor");
    return ESP_OK;

}

/**
 * @brief Default initialization with reasonable settings:
 *        1× oversampling for all, normal mode, 125 ms standby, no filter.
 */
esp_err_t bme280_init_default(bme280_handle_t bme280_sensor) {

    const bme280_device_config_t default_cfg = {
        .measurement_mode         = BME280_MODE_NORMAL,
        .temperature_oversampling = BME280_OVERSAMPLING_X1,
        .pressure_oversampling    = BME280_OVERSAMPLING_X1,
        .humidity_oversampling    = BME280_OVERSAMPLING_X1,
        .filter_mode              = BME280_FILTER_OFF,
        .standby_duration         = BME280_STANDBY_MS_125
    };

    return bme280_init(bme280_sensor, &default_cfg);

}

/**
 * @brief Delete sensor handle, remove from I2C bus and free memory.
 */
esp_err_t bme280_delete(bme280_handle_t * const bme280_sensor) {

    ESP_RETURN_ON_FALSE(bme280_sensor && *bme280_sensor, ESP_ERR_INVALID_ARG, API_TAG, "Invalid arguments (attempting to delete NULL)");

    if (i2c_master_bus_rm_device((*bme280_sensor)->dev_handle) != ESP_OK) {
        ESP_LOGW(API_TAG, "Failed to remove the device from the bus");
    }

    free(*bme280_sensor);
    *bme280_sensor = NULL;

    return ESP_OK;

}

/**
 * @brief Perform a measurement and return calibrated physical values (°C, hPa, %RH).
 *
 * For forced mode: triggers a new measurement.
 * Waits for completion, reads raw ADC values, applies compensation formulas.
 */
esp_err_t bme280_measure(bme280_const_handle_t bme280_sensor, bme280_measurement_t * const output_handle) {

    ESP_RETURN_ON_FALSE(bme280_sensor && output_handle, ESP_ERR_INVALID_ARG, API_TAG, "Invalid arguments");
    ESP_RETURN_ON_FALSE(bme280_sensor->ctrl_meas.mode > BME280_MODE_SLEEP, ESP_ERR_INVALID_STATE, API_TAG, "Cannot measure in sleep mode");

    esp_err_t ret;
    uint8_t raw_data[8];

    // In forced mode, trigger measurement by writing mode bits
    if (bme280_sensor->ctrl_meas.mode == BME280_MODE_FORCED || bme280_sensor->ctrl_meas.mode == BME280_MODE_FORCED_ALT) {
        ret = bme280_write_reg_field(bme280_sensor, BME280_CTRL_MEAS_REG, &bme280_sensor->ctrl_meas);
        ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to trigger forced measurement");
    }

    ret = bme280_wait_sensor_ready(bme280_sensor, pdMS_TO_TICKS(BME280_TIMEOUT_MS_DEFAULT));
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Measurement timeout");

    // Burst read pressure (0xF7) + temperature (0xFA) + humidity (0xFD) = 8 bytes
    ret = bme280_read_reg_many(bme280_sensor, BME280_PRESSURE_REG, 8, raw_data);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to read measurement registers");

    // Reconstruct 20-bit temperature, 20-bit pressure, 16-bit humidity
    int32_t adc_T = (int32_t)((raw_data[3] << 12) | (raw_data[4] << 4) | (raw_data[5] >> 4));
    int32_t adc_P = (int32_t)((raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4));
    int32_t adc_H = (int32_t)(((int32_t)raw_data[6] << 8) | raw_data[7]);

    int32_t t_fine;
    output_handle->temperature = compensate_temperature(bme280_sensor, adc_T, &t_fine) / 100.0;
    output_handle->pressure    = compensate_pressure(bme280_sensor, adc_P, t_fine) / 100.0;
    output_handle->humidity    = compensate_humidity(bme280_sensor, adc_H, t_fine) / 1024.0;

    return ESP_OK;

}

/* -------------------------------------------------------------------------
 * Private helper implementations
 * ------------------------------------------------------------------------- */

/**
 * @brief Read a single 8-bit register from the BME280.
 *
 * Performs a standard I2C write-then-read transaction (register address → read 1 byte).
 *
 * @param[in] bme280_sensor Valid sensor handle
 * @param[in] reg Register address to read
 * @param[out] resp Pointer to store the response
 * @return ESP_OK on success, or ESP_ERR_* on I2C failure/timeout
 */
static inline esp_err_t bme280_read_reg(bme280_const_handle_t bme280_sensor, const uint8_t reg, uint8_t * const resp) {
    return i2c_master_transmit_receive(bme280_sensor->dev_handle, &reg, 1, resp, 1, BME280_TIMEOUT_MS_DEFAULT);
}

/**
 * @brief Burst read multiple bytes starting from a given register.
 *
 * Uses the BME280's auto-increment feature for efficient sequential reads.
 *
 * @param[in] bme280_sensor Valid sensor handle
 * @param[in] start_reg Starting register address
 * @param[in] count Number of bytes to read
 * @param[out] resp Buffer large enough to hold count bytes long response
 * @return ESP_OK on success, or ESP_ERR_* on I2C failure/timeout
 */
static inline esp_err_t bme280_read_reg_many(bme280_const_handle_t bme280_sensor, const uint8_t start_reg, const size_t count, uint8_t * const resp) {
    return i2c_master_transmit_receive(bme280_sensor->dev_handle, &start_reg, 1, resp, count, BME280_TIMEOUT_MS_DEFAULT);
}

/**
 * @brief Write a single 8-bit value to a register.
 *
 * Simple I2C write transaction: register address followed by the value.
 *
 * @param[in] bme280_sensor Valid sensor handle
 * @param[in] reg Target register address
 * @param[in] value Byte value to write
 * @return ESP_OK on success, or ESP_ERR_* on I2C failure/timeout
 */
static inline esp_err_t bme280_write_reg_raw(bme280_const_handle_t bme280_sensor, const uint8_t reg, const uint8_t value) {
    const uint8_t buffer[2] = { reg, value };
    return i2c_master_transmit(bme280_sensor->dev_handle, buffer, sizeof(buffer), BME280_TIMEOUT_MS_DEFAULT);
}

/**
 * @brief Write a full 8-bit register value from a bitfield struct.
 *
 * Copies the bitfield struct (assumed to be exactly 1 byte) and writes it as-is.
 * Used to write complete control/configuration registers (CTRL_MEAS, CTRL_HUM, CONFIG).
 *
 * @param[in] bme280_sensor Valid sensor handle
 * @param[in] reg_addr Target register address
 * @param[in] field_ptr Pointer to a 1-byte bitfield struct (e.g. &sensor->ctrl_meas)
 * @return ESP_OK on success, or ESP_ERR_* on I2C failure/timeout
 */
static inline esp_err_t bme280_write_reg_field(bme280_const_handle_t bme280_sensor, const uint8_t reg_addr, const void * const field_ptr) {
    uint8_t value;
    memcpy(&value, field_ptr, 1);   // Necessary due to the bit field approach
    return bme280_write_reg_raw(bme280_sensor, reg_addr, value);
}

/**
 * @brief Poll status register until no measuring or im_update flags are set.
 */
static inline esp_err_t bme280_wait_sensor_ready(bme280_const_handle_t bme280_sensor, const TickType_t timeout_ticks) {

    esp_err_t ret;
    bme280_status_t status;

    const TickType_t start = xTaskGetTickCount();

    do {
        if (xTaskGetTickCount() - start >= timeout_ticks) {
            return ESP_ERR_TIMEOUT;
        }

        ret = bme280_read_reg(bme280_sensor, BME280_STATUS_REG, (uint8_t*)&status);
        if (ret != ESP_OK) return ret;

        vTaskDelay(pdMS_TO_TICKS(2)); // Short back-off, not to overwhelm the I2C bus (approx. shortest possible response time)

    } while (status.im_update || status.measuring);

    return ESP_OK;

}

static inline esp_err_t bme280_assign_config_params(bme280_handle_t bme280_sensor, const bme280_device_config_t * const bme280_device_config) {

    ESP_RETURN_ON_FALSE(bme280_sensor && bme280_device_config, ESP_ERR_INVALID_ARG, API_TAG, "Invalid arguments");

    // Basic range validation
    ESP_RETURN_ON_FALSE(
        bme280_device_config->temperature_oversampling <= BME280_OVERSAMPLING_X16 &&
        bme280_device_config->pressure_oversampling    <= BME280_OVERSAMPLING_X16 &&
        bme280_device_config->humidity_oversampling    <= BME280_OVERSAMPLING_X16 &&
        bme280_device_config->measurement_mode         <= BME280_MODE_NORMAL &&
        bme280_device_config->filter_mode              <= BME280_FILTER_X16 &&
        bme280_device_config->standby_duration         <= BME280_STANDBY_MS_20,   /* Note: the highest numeric value of the standby duration parameter
                                                                                   * does not correspond to the highest true standby duration time */
        ESP_ERR_INVALID_ARG, API_TAG, "Invalid configuration parameter value"
    );

    bme280_sensor->ctrl_meas.osrs_t = bme280_device_config->temperature_oversampling;
    bme280_sensor->ctrl_meas.osrs_p = bme280_device_config->pressure_oversampling;
    bme280_sensor->ctrl_meas.mode   = bme280_device_config->measurement_mode;
    bme280_sensor->ctrl_hum.osrs_h  = bme280_device_config->humidity_oversampling;
    bme280_sensor->config.filter    = bme280_device_config->filter_mode;
    bme280_sensor->config.t_sb      = bme280_device_config->standby_duration;
    bme280_sensor->config.spi3w_en  = 0; // Always disable SPI

    return ESP_OK;

}

/**
 * @brief Load factory calibration coefficients from sensor NVM.
 */
static inline esp_err_t bme280_load_calibration_data(bme280_handle_t bme280_sensor) {

    ESP_RETURN_ON_FALSE(bme280_sensor, ESP_ERR_INVALID_ARG, API_TAG, "Invalid arguments");

    esp_err_t ret;
    uint8_t hc_buf[BME280_CALIB_HUMIDITY_REG_COUNT - 1] = {0};

    // Burst read temperature & pressure calibration (0x88 → 0x9F, 24 bytes)
    ret = bme280_read_reg_many(bme280_sensor, BME280_CALIB_TEMP_PRES_FIRST_REG, BME280_CALIB_TEMP_PRESS_REG_COUNT, bme280_sensor->tp_calib.raw);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to read temp/pressure calibration");

    // Humidity calibration: H1 separate, then H2-H6 (non-continuous)
    ret = bme280_read_reg(bme280_sensor, BME280_CALIB_HUMIDITY_FIRST_REG, &bme280_sensor->h_calib.dig_H1);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to read dig_H1");
    ret = bme280_read_reg_many(bme280_sensor, BME280_CALIB_HUMIDITY_SECOND_REG, BME280_CALIB_HUMIDITY_REG_COUNT - 1, hc_buf);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to read humidity calibration block");

    bme280_sensor->h_calib.dig_H2 = (int16_t)(hc_buf[0] | ((uint16_t)hc_buf[1] << 8));
    bme280_sensor->h_calib.dig_H3 = hc_buf[2];
    // dig_H4: upper 8 bits in hc_buf[3], lower 4 in hc_buf[4] low nibble
    bme280_sensor->h_calib.dig_H4 = (int16_t)((int8_t)hc_buf[3] << 4 | (hc_buf[4] & 0x0F));
    // dig_H5: upper 4 bits in hc_buf[4] high nibble, lower 8 in hc_buf[5]
    bme280_sensor->h_calib.dig_H5 = (int16_t)((int8_t)hc_buf[5] << 4 | (hc_buf[4] >> 4));
    bme280_sensor->h_calib.dig_H6 = (int8_t)hc_buf[6];

    return ESP_OK;

}

/* -------------------------------------------------------------------------
 * Official Bosch compensation formulas
 * Source: Bosch Sensortec BME280 datasheet / reference code (as per appendix A)
 * Link: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
 * ------------------------------------------------------------------------- */

int32_t compensate_temperature(bme280_const_handle_t bme280_sensor, const int32_t adc_T, int32_t * const T_fine) {

    int32_t var1, var2, T;

    var1 = (((adc_T >> 3) - ((int32_t)bme280_sensor->tp_calib.calib.dig_T1 << 1)) * ((int32_t)bme280_sensor->tp_calib.calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bme280_sensor->tp_calib.calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bme280_sensor->tp_calib.calib.dig_T1))) >> 12)
            * ((int32_t)bme280_sensor->tp_calib.calib.dig_T3)) >> 14;

    *T_fine = var1 + var2;
    T = (*T_fine * 5 + 128) >> 8;

    return T;

}

uint32_t compensate_pressure(bme280_const_handle_t bme280_sensor, const int32_t adc_P, const int32_t T_fine) {

    int32_t var1, var2;
    uint32_t P;

    var1 = (((int32_t)T_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)bme280_sensor->tp_calib.calib.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)bme280_sensor->tp_calib.calib.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)bme280_sensor->tp_calib.calib.dig_P4) << 16);
    var1 = (((bme280_sensor->tp_calib.calib.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3)
            + ((((int32_t)bme280_sensor->tp_calib.calib.dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)bme280_sensor->tp_calib.calib.dig_P1)) >> 15);

    // Avoid division by zero
    if (var1 == 0) {
        return 0;
    }

    P = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;

    if (P < 0x80000000UL) {
        P = (P << 1) / ((uint32_t)var1);
    } else {
        P = (P / (uint32_t)var1) * 2;
    }

    var1 = (((int32_t)bme280_sensor->tp_calib.calib.dig_P9) * ((int32_t)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(P >> 2)) * ((int32_t)bme280_sensor->tp_calib.calib.dig_P8)) >> 13;
    P = (uint32_t)((int32_t)P + ((var1 + var2 + bme280_sensor->tp_calib.calib.dig_P7) >> 4));

    return P;

}

uint32_t compensate_humidity(bme280_const_handle_t bme280_sensor, const int32_t adc_H, const int32_t T_fine) {

    int32_t v_x1_u32r = T_fine - ((int32_t)76800);

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme280_sensor->h_calib.dig_H4) << 20)
                   - (((int32_t)bme280_sensor->h_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15)
                * (((((((v_x1_u32r * ((int32_t)bme280_sensor->h_calib.dig_H6)) >> 10)
                       * (((v_x1_u32r * ((int32_t)bme280_sensor->h_calib.dig_H3)) >> 11)
                           + ((int32_t)32768))) >> 10)
                      + ((int32_t)2097152)) * ((int32_t)bme280_sensor->h_calib.dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)bme280_sensor->h_calib.dig_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t)(v_x1_u32r >> 12);

}
