/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "../../bmi3.h"

/******************************************************************************/
/*!                Macro definition                                           */

#define READ_WRITE_LEN  UINT8_C(8)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                Static function definition                                 */

/*!
 * @brief This internal API reads I2C function map to COINES platform
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     len      : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks.
 *
 * @return Status of execution.
 */
static BMI3_INTF_RET_TYPE bmi3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief This internal API writes I2C function map to COINES platform
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     len      : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks.
 *
 * @return Status of execution.
 */
static BMI3_INTF_RET_TYPE bmi3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief This internal API reads SPI function map to COINES platform
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     len      : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks.
 *
 * @return Status of execution.
 */
static BMI3_INTF_RET_TYPE bmi3_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief This internal API writes SPI function map to COINES platform
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     len      : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks.
 *
 * @return Status of execution.
 */
static BMI3_INTF_RET_TYPE bmi3_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief This internal API maps delay function to COINES platform
 *
 * @param[in] period       : The time period in microseconds
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 *
 * @return void.
 */
static void bmi3_delay_us(uint32_t period, void *intf_ptr);

/******************************************************************************/
/*!               User interface functions                                    */

/*!
 * @brief This API prints the execution status
 */
void bmi3_error_codes_print_result(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMI3_OK:

            /*! Do nothing */
            break;

        case BMI3_E_NULL_PTR:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer," " which has been initialized to NULL.\r\n",
                rslt);
            break;

        case BMI3_E_COM_FAIL:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due " "to power failure during communication\r\n",
                rslt);
            break;

        case BMI3_E_DEV_NOT_FOUND:
            printf("%s\t", api_name);
            printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                   rslt);
            break;

        case BMI3_E_INVALID_SENSOR:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the " "available one\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_INT_PIN:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins " "apart from INT1 and INT2\r\n",
                rslt);
            break;

        case BMI3_E_ACC_INVALID_CFG:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration" " register which could be one among range, BW or filter performance in reg address 0x20\r\n",
                rslt);
            break;

        case BMI3_E_GYRO_INVALID_CFG:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration" "register which could be one among range, BW or filter performance in reg address 0x21\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_INPUT:
            printf("%s\t", api_name);
            printf("Error [%d] : Invalid input error. It occurs when the sensor input validity fails\r\n", rslt);
            break;

        case BMI3_E_INVALID_STATUS:
            printf("%s\t", api_name);
            printf("Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails\r\n", rslt);
            break;

        case BMI3_E_DATA_RDY_INT_FAILED:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit " "and data ready status is not updated\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_FOC_POSITION:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong" " axes\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_ST_SELECTION:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid self-test selection error. It occurs when there is an invalid precondition" "settings such as alternate accelerometer and gyroscope enable bits, accelerometer mode and output data rate\r\n",
                rslt);
            break;

        case BMI3_E_OUT_OF_RANGE:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Out of range error. It occurs when the range exceeds the maximum range for accel while performing FOC\r\n",
                rslt);
            break;

        case BMI3_E_FEATURE_ENGINE_STATUS:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Feature engine status error. It occurs when the feature engine enable mask is not set\r\n",
                rslt);
            break;

        default:
            printf("%s\t", api_name);
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}

