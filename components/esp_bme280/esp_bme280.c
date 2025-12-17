#include "esp_bme280.h"


int32_t compensate_temperature(const bme280_sensor_t * const sensor, const int32_t adc_T, int32_t * const T_fine) {

    int32_t var1, var2, T;

    var1 = (((adc_T >> 3) - ((int32_t)sensor->calib.dig_T1 << 1)) * ((int32_t)sensor->calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)sensor->calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)sensor->calib.dig_T1))) >> 12)
        * ((int32_t)sensor->calib.dig_T3)) >> 14;

    *T_fine = var1 + var2;
    T = (*T_fine * 5 + 128) >> 8;

    return T;

}
