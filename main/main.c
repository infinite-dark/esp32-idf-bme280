#include <esp_log.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_bme280.h"

#define I2C_PIN_SDA     42
#define I2C_PIN_SCL     41
#define I2C_MASTER_NUM   0

static const char *TAG = "bme280_test";

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
    bme280_device_config_t config = {
        .measurement_mode = BME280_MODE_NORMAL,
        .temperature_oversampling = BME280_OVERSAMPLING_X16,
        .pressure_oversampling = BME280_OVERSAMPLING_X1,
        .humidity_oversampling = BME280_OVERSAMPLING_X1,
        .filter_mode = BME280_FILTER_X8,
        .standby_duration = BME280_STANDBY_MS_20
    };

    ESP_ERROR_CHECK(bme280_create_default(bus_handle, BME280_I2C_ADDRESS_DEFAULT, &sensor));
    ESP_ERROR_CHECK(bme280_init(sensor, &config));

    ESP_LOGI(TAG, "Starting measurement loop...");

    while (1) {
        int32_t adc_T = simple_test(sensor);
        int32_t t_fine;
        int32_t temp_final = compensate_temperature(sensor, adc_T, &t_fine);
        printf("Temp: %.2f °C\n", temp_final/100.0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (sensor != NULL) {
        bme280_delete(&sensor);
    }

}