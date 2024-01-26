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

static const char *TAG = "VRGloveControllerBMI323SelfTest";

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int self_test(struct bmi3_dev *dev)
{
    /* Variable to define error. */
    int8_t rslt;

    uint8_t idx;

    uint8_t limit = 3;

    /* Array to define self-test modes. */
    uint8_t self_test_selection[3] = { BMI3_ST_ACCEL_ONLY, BMI3_ST_GYRO_ONLY, BMI3_ST_BOTH_ACC_GYR };

    struct bmi3_st_result st_result_status = { 0 };

            for (idx = 0; idx < limit; idx++)
            {
                if ((idx + 1) == BMI3_ST_ACCEL_ONLY)
                {
                    ESP_LOGI(TAG, "Self-test for accel only");
                }
                else if ((idx + 1) == BMI3_ST_GYRO_ONLY)
                {
                    ESP_LOGI(TAG, "Self-test for gyro only");
                }
                else
                {
                    ESP_LOGI(TAG, "Self-test for both accel and gyro");
                }

                /* Performs self-test for either accel, gyro or both */
                rslt = bmi323_perform_self_test(self_test_selection[idx], &st_result_status, dev);
                bmi3_error_codes_print_result("Perform_self_test", rslt);

                if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_TRUE))
                {
                    ESP_LOGI(TAG, "Self-test is successfully completed ");
                }

                if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_FALSE))
                {
                    ESP_LOGI(TAG, "Self-test is not successfully completed");

                    switch (st_result_status.self_test_err_rslt)
                    {
                        case BMI3_SC_ST_ABORTED_MASK:
                            ESP_LOGI(TAG, "SC_ST_ABORTED");
                            break;
                        case BMI3_ST_IGNORED_MASK:
                            ESP_LOGI(TAG, "BMI323_ST_IGNORED");
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

                ESP_LOGI(TAG, "Result of acc_x_axis is %d", st_result_status.acc_sens_x_ok);
                ESP_LOGI(TAG, "Result of acc_y_axis is %d", st_result_status.acc_sens_y_ok);
                ESP_LOGI(TAG, "Result of acc_z_axis is %d", st_result_status.acc_sens_z_ok);
                ESP_LOGI(TAG, "Result of gyr_x_axis is %d", st_result_status.gyr_sens_x_ok);
                ESP_LOGI(TAG, "Result of gyr_y_axis is %d", st_result_status.gyr_sens_y_ok);
                ESP_LOGI(TAG, "Result of gyr_z_axis is %d", st_result_status.gyr_sens_z_ok);
                ESP_LOGI(TAG, "Result of gyr_drive_ok is %d", st_result_status.gyr_drive_ok);
                ESP_LOGI(TAG, "Result of self-test error is %d", st_result_status.self_test_err_rslt);
                ESP_LOGI(TAG, "Result of ST_result is %d", st_result_status.self_test_rslt);
            }

    return rslt;
}
