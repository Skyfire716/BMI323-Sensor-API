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

static const char *TAG = "VRGloveControllerBMI323SelfCalibration";


/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations for accel.
 *
 *  @param[in] dev       : Structure instance of bmi3_dev.
 *
 *  @return Status of execution.
 */
static int8_t bmi3_set_accel_config(struct bmi3_dev *dev);

/*!
 *  @brief This internal API is used to set configurations for gyro.
 *
 *  @param[in] dev       : Structure instance of bmi3_dev.
 *
 *  @return Status of execution.
 */
static int8_t bmi3_set_gyro_config(struct bmi3_dev *dev);

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int self_calibration(struct bmi3_dev *dev)
{
    /* Structure to define the self-calibration result and error result */
    struct bmi3_self_calib_rslt sc_rslt;

    /* Variable to choose apply correction */
    uint8_t apply_corr = BMI3_SC_APPLY_CORR_EN;

    /* Variable to define error. */
    int8_t rslt;

    /* Variable to define index. */
    uint8_t idx;

    /* Variable to define limit. */
    uint8_t limit = 2;

    /* Array to define self-calibration modes. */
    uint8_t sc_selection[2] = { BMI3_SC_SENSITIVITY_EN, BMI3_SC_OFFSET_EN };

    /* Structure instance of gyro dp gain offset */
    struct bmi3_gyr_dp_gain_offset gyr_dp_gain_offset = { 0 };

        rslt = bmi3_set_accel_config(dev);
        bmi3_error_codes_print_result("set_accel_config", rslt);

        rslt = bmi3_set_gyro_config(dev);
        bmi3_error_codes_print_result("set_gyro_config", rslt);

            for (idx = 0; idx < limit; idx++)
            {
                if ((idx + 1) == BMI3_SC_SENSITIVITY_EN)
                {
                    ESP_LOGI(TAG, "Self-calibration for sensitivity mode");
                }
                else
                {
                    ESP_LOGI(TAG, "Self-calibration for offset mode");
                }

                /* Performs self-calibration for either sensitivity, offset or both */
                rslt = bmi323_perform_gyro_sc(sc_selection[idx], apply_corr, &sc_rslt, dev);
                bmi3_error_codes_print_result("bmi323_perform_gyro_sc", rslt);

                if ((rslt == BMI323_OK) && (sc_rslt.gyro_sc_rslt == BMI323_TRUE))
                {
                    ESP_LOGI(TAG, "Self calibration is successfully completed ");
                }

                if ((rslt == BMI323_OK) && (sc_rslt.gyro_sc_rslt == BMI323_FALSE))
                {
                    ESP_LOGI(TAG, "Self calibration is not successfully completed ");

                    switch (sc_rslt.sc_error_rslt)
                    {
                        case BMI3_SC_ST_ABORTED_MASK:
                            ESP_LOGI(TAG, "SC_ST_ABORTED");
                            break;
                        case BMI3_SC_ST_PRECON_ERR_MASK:
                            ESP_LOGI(TAG, "BMI323_SC_ST_PRECON_ERR");
                            break;
                        case BMI3_MODE_CHANGE_WHILE_SC_ST_MASK:
                            ESP_LOGI(TAG, "BMI323_MODE_CHANGE_WHILE_SC_ST");
                            break;
                        default:
                            break;
                    }
                }

                ESP_LOGI(TAG, "Result of self-calibration error ");

                if (((sc_rslt.sc_error_rslt & BMI3_SET_LOW_NIBBLE) == BMI3_NO_ERROR_MASK))
                {
                    ESP_LOGI(TAG, "\tNo error");
                }
                else
                {
                    ESP_LOGI(TAG, "\tError: %d", sc_rslt.sc_error_rslt & BMI3_SET_LOW_NIBBLE);
                }

                if (sc_rslt.sc_error_rslt & BMI3_SC_ST_COMPLETE_MASK)
                {
                    ESP_LOGI(TAG, "\tSelf-calibration procedure is completed.");
                }
                else
                {
                    ESP_LOGI(TAG, "\tError: Self-calibration procedure is not completed");
                }

                ESP_LOGI(TAG, "Result of self-calibration is %d", sc_rslt.gyro_sc_rslt);

                rslt = bmi323_get_gyro_dp_off_dgain(&gyr_dp_gain_offset, dev);
                bmi3_error_codes_print_result("bmi323_get_gyro_off_dgain", rslt);
                ESP_LOGI(TAG, "Result of gyro dp offset x is %d", gyr_dp_gain_offset.gyr_dp_off_x);
                ESP_LOGI(TAG, "Result of gyro dp offset y is %d", gyr_dp_gain_offset.gyr_dp_off_y);
                ESP_LOGI(TAG, "Result of gyro dp offset z is %d", gyr_dp_gain_offset.gyr_dp_off_z);
                ESP_LOGI(TAG, "Result of gyro dp gain x is %d", gyr_dp_gain_offset.gyr_dp_dgain_x);
                ESP_LOGI(TAG, "Result of gyro dp gain y is %d", gyr_dp_gain_offset.gyr_dp_dgain_y);
                ESP_LOGI(TAG, "Result of gyro dp gain z is %d", gyr_dp_gain_offset.gyr_dp_dgain_z);
            }

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for accel.
 */
static int8_t bmi3_set_accel_config(struct bmi3_dev *dev)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer configuration. */
    struct bmi3_sens_config config;

    /* Structure to map interrupt */
    struct bmi3_map_int map_int = { 0 };

    /* Configure the type of feature. */
    config.type = BMI323_ACCEL;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(&config, 1, dev);
    bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);

    if (rslt == BMI323_OK)
    {
        map_int.acc_drdy_int = BMI3_INT1;

        /* Map data ready interrupt to interrupt pin. */
        rslt = bmi323_map_interrupt(map_int, dev);
        bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);

        if (rslt == BMI323_OK)
        {
            /* NOTE: The user can change the following configuration parameters according to their requirement. */
            /* Output Data Rate. By default ODR is set as 100Hz for accel. */
            config.cfg.acc.odr = BMI3_ACC_ODR_100HZ;

            /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
            config.cfg.acc.range = BMI3_ACC_RANGE_8G;

            /* The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR. */
            config.cfg.acc.bwp = BMI3_ACC_BW_ODR_HALF;

            /* Set number of average samples for accel. */
            config.cfg.acc.avg_num = BMI3_ACC_AVG1;

            /* Enable the accel mode where averaging of samples
             * will be done based on above set bandwidth and ODR.
             * Note : By default accel is disabled. The accel will get enable by selecting the mode.
             */
            config.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

            /* Set the accel configurations. */
            rslt = bmi323_set_sensor_config(&config, 1, dev);
            bmi3_error_codes_print_result("bmi323_set_sensor_config", rslt);
        }
    }

    return rslt;
}

/*!
 *  @brief This internal API is used to set configurations for gyro.
 */
static int8_t bmi3_set_gyro_config(struct bmi3_dev *dev)
{
    struct bmi3_map_int map_int = { 0 };

    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi3_sens_config config;

    /* Configure the type of feature. */
    config.type = BMI323_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(&config, 1, dev);
    bmi3_error_codes_print_result("Get sensor config", rslt);

    if (rslt == BMI323_OK)
    {
        map_int.gyr_drdy_int = BMI3_INT1;

        /* Map data ready interrupt to interrupt pin. */
        rslt = bmi323_map_interrupt(map_int, dev);
        bmi3_error_codes_print_result("Map interrupt", rslt);

        if (rslt == BMI323_OK)
        {
            /* The user can change the following configuration parameters according to their requirement. */
            /* Output Data Rate. By default ODR is set as 100Hz for gyro. */
            config.cfg.gyr.odr = BMI3_GYR_ODR_100HZ;

            /* Gyroscope Angular Rate Measurement Range. By default the range is 2000dps. */
            config.cfg.gyr.range = BMI3_GYR_RANGE_500DPS;

            /*  The Gyroscope bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR
             *  Value   Name      Description
             *    0   odr_half   BW = gyr_odr/2
             *    1  odr_quarter BW = gyr_odr/4
             */
            config.cfg.gyr.bwp = BMI3_GYR_BW_ODR_HALF;

            /* By default the gyro is disabled. Gyro is enabled by selecting the mode. */
            config.cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;

            /* Value    Name    Description
             *  000     avg_1   No averaging; pass sample without filtering
             *  001     avg_2   Averaging of 2 samples
             *  010     avg_4   Averaging of 4 samples
             *  011     avg_8   Averaging of 8 samples
             *  100     avg_16  Averaging of 16 samples
             *  101     avg_32  Averaging of 32 samples
             *  110     avg_64  Averaging of 64 samples
             */
            config.cfg.gyr.avg_num = BMI3_GYR_AVG1;

            /* Set the gyro configurations. */
            rslt = bmi323_set_sensor_config(&config, 1, dev);
            bmi3_error_codes_print_result("Set sensor config", rslt);
        }
    }

    return rslt;
}
