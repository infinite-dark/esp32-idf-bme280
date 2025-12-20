#include "esp_bme280.h"
#include <esp_check.h>


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

    if (bus_handle == NULL || dev_cfg == NULL || out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    i2c_master_dev_handle_t dev_handle;
    ret = i2c_master_bus_add_device(bus_handle, dev_cfg, &dev_handle);
    ESP_GOTO_ON_ERROR(ret, err_return, API_TAG, "Failed to add the device to the I2C bus");

    ret = i2c_master_probe(bus_handle, dev_cfg->device_address, 100);
    ESP_GOTO_ON_ERROR(ret, err_release, API_TAG, "Failed to find the BME280 at address 0x%x", dev_cfg->device_address);

    struct bme280_sensor * sensor = calloc(1, sizeof(*sensor));
    if (!sensor) {
        ESP_LOGE(API_TAG, "Failed to allocate the BME280 sensor context");
        ret = ESP_ERR_NO_MEM;
        goto err_release;
    }

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
