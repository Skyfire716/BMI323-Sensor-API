/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "../../bmi323.h"
#include "BMI323_Wrapper.h"
#include "esp_log.h"

static const char *TAG = "VRGloveControllerBMI323Temperature";

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int temperature(struct bmi3_dev *dev, float *temperature_value)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;


    /* Variable to define limit to print temperature data. */
    uint16_t limit = 100;

    /* Structure to define accelerometer configuration. */
    struct bmi3_sens_config config = { 0 };

    /* Structure to define interrupt with its feature */
    struct bmi3_map_int map_int = { 0 };

    /* Variable to store temperature value */
    uint16_t temperature_data;

    /* Initialize the interrupt status of temperature */
    uint16_t int_status;

    /* Variable to store sensor time. */
    uint32_t sensor_time = 0;

    uint8_t indx = 0;
            /* Get default configurations for the type of feature selected. */
            rslt = bmi323_get_sensor_config(&config, 1, dev);
            bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);

            /* Configure the type of feature. */
            config.type = BMI323_ACCEL;

            /* Enable accel by selecting mode. */
            config.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

            if (rslt == BMI323_OK)
            {
                rslt = bmi323_set_sensor_config(&config, 1, dev);
                bmi3_error_codes_print_result("bmi323_set_sensor_config", rslt);

                if (rslt == BMI323_OK)
                {
                    /* Select the feature and map the interrupt to pin BMI323_INT1 or BMI323_INT2. */
                    map_int.temp_drdy_int = BMI3_INT1;

                    /* Map temperature data ready interrupt to interrupt pin. */
                    rslt = bmi323_map_interrupt(map_int, dev);
                    bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);

                    ESP_LOGI(TAG, "TEMP_DATA_SET, Temperature data (Degree celcius), SensorTime(secs)");

                    while (indx <= limit)
                    {
                        /* To get the status of temperature data ready interrupt. */
                        rslt = bmi323_get_int1_status(&int_status, dev);
                        bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);

                        if (int_status & BMI3_INT_STATUS_TEMP_DRDY)
                        {
                            /* Get temperature data. */
                            rslt = bmi323_get_temperature_data(&temperature_data, dev);
                            bmi3_error_codes_print_result("bmi323_get_sensor_data", rslt);

                            *temperature_value = (float)((((float)((int16_t)temperature_data)) / 512.0) + 23.0);

                            rslt = bmi323_get_sensor_time(&sensor_time, dev);
                            bmi3_error_codes_print_result("bmi323_get_sensor_data", rslt);

                            ESP_LOGI(TAG, "%d, %f, %.4lf",
                                   indx,
                                   *temperature_value,
                                   (sensor_time * BMI3_SENSORTIME_RESOLUTION));

                            indx++;
                        }
                    }
                }
            }

    return rslt;
}
