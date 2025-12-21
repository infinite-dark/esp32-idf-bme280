#include <esp_log.h>
#include <stdio.h>
#include "esp_bme280.h"

#define I2C_PIN_SDA     42
#define I2C_PIN_SCL     41
#define I2C_MASTER_NUM   0


static const char *TAG = "i2c example";

void app_main(void) {

    i2c_master_bus_handle_t bus_handle;
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_PIN_SDA,
        .scl_io_num = I2C_PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    bme280_handle_t sensor = NULL;
    bme280_device_config_t config = {0};

    ESP_ERROR_CHECK(bme280_create_default(bus_handle, BME280_I2C_ADDRESS_DEFAULT, &sensor));
    ESP_ERROR_CHECK(bme280_init(sensor, &config));
    if (sensor != NULL) {
        bme280_delete(&sensor);
    }

}
