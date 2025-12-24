#include <stdlib.h>
#include "esp_bme280.h"
#include <esp_check.h>
#include <freertos/FreeRTOS.h>

#define BME280_TIMEOUT_MS_DEFAULT     100

static const char* const API_TAG = "bme280_api";

struct bme280_sensor {
    i2c_master_dev_handle_t     dev_handle;
    bme280_ctrl_meas_t          ctrl_meas;
    bme280_ctrl_hum_t           ctrl_hum;
    bme280_config_t             config;
    bme280_calib_temp_press_t   tp_calib;
    bme280_calib_humidity_t     h_calib;
};


esp_err_t bme280_create(i2c_master_bus_handle_t bus_handle, const i2c_device_config_t * const dev_cfg, bme280_handle_t * const out_handle) {

    ESP_LOGI(API_TAG, "Attempting to create BME280 sensor context...");

    ESP_RETURN_ON_FALSE(bus_handle && dev_cfg && out_handle, ESP_ERR_INVALID_ARG, API_TAG, "Invalid arguments");

    esp_err_t ret;

    ret = i2c_master_probe(bus_handle, dev_cfg->device_address, BME280_TIMEOUT_MS_DEFAULT);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to find the BME280 at address 0x%x", dev_cfg->device_address);

    i2c_master_dev_handle_t dev_handle;
    ret = i2c_master_bus_add_device(bus_handle, dev_cfg, &dev_handle);
    ESP_GOTO_ON_ERROR(ret, err_return, API_TAG, "Failed to add the device to the I2C bus");

    struct bme280_sensor * sensor = calloc(1, sizeof(struct bme280_sensor));
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

esp_err_t bme280_create_default(i2c_master_bus_handle_t bus_handle, const uint8_t dev_addr, bme280_handle_t * const out_handle) {

    if (dev_addr != BME280_I2C_ADDRESS_DEFAULT && dev_addr != BME280_I2C_ADDRESS_ALTER) {
        ESP_LOGW(API_TAG, "Uncommon BME280 I2C address 0x%02x", dev_addr);
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = 400000,
        .scl_wait_us = 0,
        .flags = { .disable_ack_check = false }
    };

    return bme280_create(bus_handle, &dev_cfg, out_handle);

}

static inline esp_err_t bme280_read_reg(bme280_const_handle_t bme280_sensor, const uint8_t reg, uint8_t * const resp) {

    return i2c_master_transmit_receive(bme280_sensor->dev_handle, &reg, 1, resp, 1, BME280_TIMEOUT_MS_DEFAULT);

}

static inline esp_err_t bme280_read_reg_many(bme280_const_handle_t bme280_sensor, const uint8_t start_reg, const size_t count, uint8_t * const resp) {

    return i2c_master_transmit_receive(bme280_sensor->dev_handle, &start_reg, 1, resp, count, BME280_TIMEOUT_MS_DEFAULT);

}

static inline esp_err_t bme280_write_reg_raw(bme280_const_handle_t bme280_sensor, const uint8_t reg, const uint8_t value) {

    const uint8_t buffer[2] = { reg, value };
    return i2c_master_transmit(bme280_sensor->dev_handle, buffer, sizeof(buffer), BME280_TIMEOUT_MS_DEFAULT);

}

static inline esp_err_t bme280_write_reg_field(bme280_const_handle_t bme280_sensor, const uint8_t reg_addr, const void * const field_ptr, const char * const field_name) {

    uint8_t value;
    memcpy(&value, field_ptr, 1);

    return bme280_write_reg_raw(bme280_sensor, reg_addr, value);

}

static inline esp_err_t bme280_wait_sensor_ready(bme280_const_handle_t bme280_sensor, const TickType_t timeout_ticks) {

    esp_err_t ret;
    bme280_status_t status;

    const TickType_t start = xTaskGetTickCount();

    do {

        if (xTaskGetTickCount() - start >= timeout_ticks) {
            return ESP_ERR_TIMEOUT;
        }

        ret = bme280_read_reg(bme280_sensor, BME280_STATUS_REG, (uint8_t*)&status);
        if (ret != ESP_OK) {
            return ret;
        }

        vTaskDelay(pdMS_TO_TICKS(2));

    } while (status.im_update || status.measuring);

    return ESP_OK;

}

static inline esp_err_t bme280_assign_config_params(bme280_handle_t bme280_sensor, const bme280_device_config_t * const bme280_device_config) {

    ESP_RETURN_ON_FALSE(bme280_sensor && bme280_device_config, ESP_ERR_INVALID_ARG, API_TAG, "Invalid arguments");

    ESP_RETURN_ON_FALSE(
            bme280_device_config->temperature_oversampling <= BME280_OVERSAMPLING_X16 &&
            bme280_device_config->pressure_oversampling <= BME280_OVERSAMPLING_X16 &&
            bme280_device_config->humidity_oversampling <= BME280_OVERSAMPLING_X16 &&
            bme280_device_config->measurement_mode <= BME280_MODE_NORMAL &&
            bme280_device_config->filter_mode <= BME280_FILTER_X16 &&
            bme280_device_config->standby_duration <= BME280_STANDBY_MS_20,
        ESP_ERR_INVALID_ARG,
        API_TAG,
        "Invalid configuration parameters"
    );

    bme280_sensor->ctrl_meas.osrs_t = bme280_device_config->temperature_oversampling;
    bme280_sensor->ctrl_meas.osrs_p = bme280_device_config->pressure_oversampling;
    bme280_sensor->ctrl_meas.mode = bme280_device_config->measurement_mode;
    bme280_sensor->ctrl_hum.osrs_h = bme280_device_config->humidity_oversampling;
    bme280_sensor->config.filter = bme280_device_config->filter_mode;
    bme280_sensor->config.t_sb = bme280_device_config->standby_duration;
    bme280_sensor->config.spi3w_en = 0;

    return ESP_OK;

}

static inline esp_err_t bme280_load_calibration_data(bme280_handle_t bme280_sensor) {

    esp_err_t ret;
    uint8_t hc_buf[BME280_CALIB_HUMIDITY_REG_COUNT - 1] = { 0 };

    ret = bme280_read_reg_many(bme280_sensor, BME280_CALIB_TEMP_PRES_FIRST_REG, BME280_CALIB_TEMP_PRESS_REG_COUNT, bme280_sensor->tp_calib.raw);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to read temperature and pressure calibration data");

    ret = bme280_read_reg(bme280_sensor, BME280_CALIB_HUMIDITY_FIRST_REG, &bme280_sensor->h_calib.dig_H1);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to read humidity calibration data");
    ret = bme280_read_reg_many(bme280_sensor, BME280_CALIB_HUMIDITY_SECOND_REG, BME280_CALIB_HUMIDITY_REG_COUNT - 1, hc_buf);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to read humidity calibration data");

    bme280_sensor->h_calib.dig_H2 = (int16_t)(hc_buf[0] | ((uint16_t)hc_buf[1] << 8));
    bme280_sensor->h_calib.dig_H3 = hc_buf[2];
    bme280_sensor->h_calib.dig_H4 = (int16_t)((int8_t)hc_buf[3] << 4 | (hc_buf[4] & 0x0F));
    bme280_sensor->h_calib.dig_H5 = (int16_t)((int8_t)hc_buf[5] << 4 | hc_buf[4] >> 4);
    bme280_sensor->h_calib.dig_H6 = (int8_t)hc_buf[6];

    return ESP_OK;

}

esp_err_t bme280_init(bme280_handle_t bme280_sensor, const bme280_device_config_t * const bme280_device_config) {

    ESP_RETURN_ON_FALSE(bme280_sensor && bme280_device_config, ESP_ERR_INVALID_ARG, API_TAG, "Invalid arguments");

    ESP_LOGI(API_TAG, "Initializing the BME280 sensor...");

    esp_err_t ret;
    uint8_t chip_id;

    ret = bme280_read_reg(bme280_sensor, BME280_CHIP_ID_REG, &chip_id);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to read chip ID");
    ESP_RETURN_ON_FALSE(chip_id == BME280_CHIP_ID_VAL, ESP_ERR_INVALID_RESPONSE, API_TAG, "Invalid chip ID");

    ret = bme280_write_reg_raw(bme280_sensor, BME280_RESET_REG, BME280_RESET_WORD);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to reset sensor");

    ret = bme280_wait_sensor_ready(bme280_sensor, pdMS_TO_TICKS(BME280_TIMEOUT_MS_DEFAULT));
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Sensor likely timed out");

    ret = bme280_load_calibration_data(bme280_sensor);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to load calibration data");

    ret = bme280_assign_config_params(bme280_sensor, bme280_device_config);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to load calibration data");

    ret = bme280_write_reg_field(bme280_sensor, BME280_CONFIG_REG, &bme280_sensor->config, "config");
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to write config");
    ret = bme280_write_reg_field(bme280_sensor, BME280_CTRL_HUM_REG, &bme280_sensor->ctrl_hum, "ctrl_hum");
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to write ctrl_hum");
    ret = bme280_write_reg_field(bme280_sensor, BME280_CTRL_MEAS_REG, &bme280_sensor->ctrl_meas, "ctrl_meas");
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to write ctrl_meas");

    ESP_LOGI(API_TAG, "Success initializing the BME280 sensor");

    return ESP_OK;

}

esp_err_t bme280_delete(bme280_handle_t * const bme280_sensor) {

    ESP_RETURN_ON_FALSE(bme280_sensor && *bme280_sensor, ESP_ERR_INVALID_ARG, API_TAG, "Invalid arguments (attempting to delete NULL)");

    if (i2c_master_bus_rm_device((*bme280_sensor)->dev_handle) != ESP_OK) {
        ESP_LOGW(API_TAG, "Failed to remove the device from the bus");
    }

    free(*bme280_sensor);
    *bme280_sensor = NULL;

    return ESP_OK;

}

int32_t simple_test(const bme280_const_handle_t bme280_sensor) {

    uint8_t raw_data[3];
    bme280_wait_sensor_ready(bme280_sensor, pdMS_TO_TICKS(BME280_TIMEOUT_MS_DEFAULT));
    bme280_read_reg_many(bme280_sensor, BME280_TEMPERATURE_REG, 3, raw_data);
    int32_t adc_T = (int32_t)((raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4));
    return adc_T;

}

int32_t compensate_temperature(const bme280_const_handle_t sensor, const int32_t adc_T, int32_t * const T_fine) {

    int32_t var1, var2, T;

    var1 = (((adc_T >> 3) - ((int32_t)sensor->tp_calib.calib.dig_T1 << 1)) * ((int32_t)sensor->tp_calib.calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)sensor->tp_calib.calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)sensor->tp_calib.calib.dig_T1))) >> 12)
        * ((int32_t)sensor->tp_calib.calib.dig_T3)) >> 14;

    *T_fine = var1 + var2;
    T = (*T_fine * 5 + 128) >> 8;

    return T;

}
