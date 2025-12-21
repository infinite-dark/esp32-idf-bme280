#include "esp_bme280.h"
#include <esp_check.h>

#define I2C_TIMEOUT_MS_DEFAULT     100

static const char* const API_TAG = "bme280_api";

struct bme280_sensor {
    i2c_master_dev_handle_t dev_handle;
    bme280_ctrl_meas_t      ctrl_meas;
    bme280_ctrl_hum_t       ctrl_hum;
    bme280_config_t         config;
    bme280_status_t         status;
    bme280_calib_t          calib;
};


esp_err_t bme280_create(i2c_master_bus_handle_t bus_handle, const i2c_device_config_t * const dev_cfg, bme280_handle_t * const out_handle) {

    ESP_RETURN_ON_FALSE(bus_handle && dev_cfg && out_handle, ESP_ERR_INVALID_ARG, API_TAG, "Invalid arguments");

    esp_err_t ret;

    i2c_master_dev_handle_t dev_handle;
    ret = i2c_master_bus_add_device(bus_handle, dev_cfg, &dev_handle);
    ESP_GOTO_ON_ERROR(ret, err_return, API_TAG, "Failed to add the device to the I2C bus");

    ret = i2c_master_probe(bus_handle, dev_cfg->device_address, I2C_TIMEOUT_MS_DEFAULT);
    ESP_GOTO_ON_ERROR(ret, err_release, API_TAG, "Failed to find the BME280 at address 0x%x", dev_cfg->device_address);

    struct bme280_sensor * sensor = calloc(1, sizeof(struct bme280_sensor));
    ESP_GOTO_ON_FALSE(sensor, ESP_ERR_NO_MEM, err_release, API_TAG, "Failed to allocate the BME280 sensor context");

    sensor->dev_handle = dev_handle;
    *out_handle = sensor;

    ESP_LOGI(API_TAG, "Success creating BME280", dev_cfg->device_address);      // TODO: fix message later

    return ESP_OK;

err_release:
    i2c_master_bus_rm_device(dev_handle);
err_return:
    ESP_LOGE(API_TAG, "Error creating BME280");     // TODO: fix message later
    return ret;

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

static inline esp_err_t bme280_read_reg(i2c_master_dev_handle_t dev, const uint8_t reg, uint8_t * const resp) {

    return i2c_master_transmit_receive(dev, &reg, 1, resp, 1, I2C_TIMEOUT_MS_DEFAULT);

}

static inline esp_err_t bme280_write_reg(i2c_master_dev_handle_t dev, const uint8_t reg, const uint8_t value) {

    const uint8_t buffer[2] = { reg, value };
    return i2c_master_transmit(dev, buffer, sizeof(buffer), I2C_TIMEOUT_MS_DEFAULT);

}

esp_err_t bme280_init(bme280_handle_t bme280_sensor, const bme280_device_config_t * const bme280_device_config) {

    ESP_RETURN_ON_FALSE(bme280_sensor && bme280_device_config, ESP_ERR_INVALID_ARG, API_TAG, "Invalid arguments");

    esp_err_t ret;

    uint8_t chip_id;
    ret = bme280_read_reg(bme280_sensor->dev_handle, BME280_CHIP_ID_REG, &chip_id);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to get chip ID");
    ESP_RETURN_ON_FALSE(chip_id == BME280_CHIP_ID_VAL, ret, API_TAG, "Invalid BME280 chip id: 0x%x (expected 0x%x)", chip_id, BME280_CHIP_ID_VAL);

    ret = bme280_write_reg(bme280_sensor->dev_handle, BME280_RESET_REG, BME280_RESET_WORD);
    ESP_RETURN_ON_ERROR(ret, API_TAG, "Failed to soft-reset");

    // TODO
    return ESP_OK;

}

esp_err_t bme280_delete(bme280_handle_t * const bme280_sensor) {

    if (bme280_sensor == NULL || *bme280_sensor == NULL) {
        ESP_LOGW(API_TAG, "Invalid argument - attempting to delete NULL");  // TODO: fix message later
        return ESP_ERR_INVALID_ARG;
    }

    i2c_master_bus_rm_device((*bme280_sensor)->dev_handle);
    free(*bme280_sensor);
    *bme280_sensor = NULL;

    return ESP_OK;

}

int32_t compensate_temperature(const bme280_const_handle_t sensor, const int32_t adc_T, int32_t * const T_fine) {

    int32_t var1, var2, T;

    var1 = (((adc_T >> 3) - ((int32_t)sensor->calib.dig_T1 << 1)) * ((int32_t)sensor->calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)sensor->calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)sensor->calib.dig_T1))) >> 12)
        * ((int32_t)sensor->calib.dig_T3)) >> 14;

    *T_fine = var1 + var2;
    T = (*T_fine * 5 + 128) >> 8;

    return T;

}
