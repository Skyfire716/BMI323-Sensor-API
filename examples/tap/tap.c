/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include "../../bmi323.h"
#include "BMI323_Wrapper.h"
#include "esp_log.h"
#include <stdio.h>

static const char *TAG = "VRGloveControllerBMI323TAP";

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations for tap interrupt.
 *
 *  @param[in] dev       : Structure instance of bmi3_dev.
 *
 *  @return Status of execution.
 */
static int8_t bmi3_set_feature_config(struct bmi3_dev *dev);

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int tap(struct bmi3_dev *dev) {
  /* Status of API are returned to this variable. */
  int8_t rslt;

  uint8_t data[2];

  /* Variable to get tap interrupt status. */
  uint16_t int_status = 0;

  /* Feature enable initialization. */
  struct bmi3_feature_enable feature = {0};

  /* Interrupt mapping structure. */
  struct bmi3_map_int map_int = {0};
  /* Set feature configurations for tap interrupt. */
  rslt = bmi3_set_feature_config(dev);
  bmi3_error_codes_print_result("Set feature config", rslt);

  if (rslt == BMI323_OK) {
    feature.tap_detector_s_tap_en = BMI323_ENABLE;
    feature.tap_detector_d_tap_en = BMI323_ENABLE;
    feature.tap_detector_t_tap_en = BMI323_ENABLE;

    /* Enable the selected sensors. */
    rslt = bmi323_select_sensor(&feature, dev);
    bmi3_error_codes_print_result("Sensor enable", rslt);

    struct bmi3_int_pin_cfg pin_cfg1 = {.lvl = 1, .od = 0, .output_en = 1};
    struct bmi3_int_pin_cfg pin_cfg2 = {.lvl = 1, .od = 0, .output_en = 1};

    const struct bmi3_int_pin_config int_cfg = {
        .pin_type = BMI3_INT1, .int_latch = 1, .pin_cfg = {pin_cfg1, pin_cfg2}};
    const struct bmi3_int_pin_config int_cfg2 = {
        .pin_type = BMI3_INT2, .int_latch = 1, .pin_cfg = {pin_cfg1, pin_cfg2}};
    rslt = bmi3_set_int_pin_config(&int_cfg, dev);
    bmi3_error_codes_print_result("Config Pins ", rslt);
    ESP_LOGI(TAG, "Conf Pins Result %u ", rslt);

    rslt = bmi3_set_int_pin_config(&int_cfg2, dev);
    bmi3_error_codes_print_result("Config Pins ", rslt);
    ESP_LOGI(TAG, "Conf Pins Result %u ", rslt);

    if (rslt == BMI323_OK) {
      map_int.tap_out = BMI3_INT2;

      /* Map the feature interrupt for tap interrupt. */
      rslt = bmi323_map_interrupt(map_int, dev);
      bmi3_error_codes_print_result("Map interrupt", rslt);
      ESP_LOGI(TAG, "Tap the board either single, double or triple tap %u ",
               rslt);

      /* Loop to get tap interrupt. */
      /*
      do
      {
      */
      /* Read the interrupt status from int 2 pin */
      /*
      rslt = bmi323_get_int2_status(&int_status, dev);
      bmi3_error_codes_print_result("Get interrupt status", rslt);
      */
      /* Check the interrupt status of the tap */
      /*
      if (int_status & BMI3_INT_STATUS_TAP)
      {
          ESP_LOGI(TAG, "Tap interrupt is generated");
          rslt = bmi323_get_regs(BMI3_REG_FEATURE_EVENT_EXT, data, 2, dev);

          if (data[0] & BMI3_TAP_DET_STATUS_SINGLE)
          {
              ESP_LOGI(TAG, "Single tap asserted");
          }

          if (data[0] & BMI3_TAP_DET_STATUS_DOUBLE)
          {
              ESP_LOGI(TAG, "Double tap asserted");
          }

          if (data[0] & BMI3_TAP_DET_STATUS_TRIPLE)
          {
              ESP_LOGI(TAG, "Triple tap asserted");
          }

          break;
      }
  } while (rslt == BMI323_OK);
  */
    }
  }
  return rslt;
}

uint8_t read_tap(struct bmi3_dev *dev) {
  int8_t rslt;

  uint16_t int_status = 0;

  uint8_t data[2];

  rslt = bmi323_get_int2_status(&int_status, dev);
  bmi3_error_codes_print_result("Get interrupt status", rslt);
  /* Check the interrupt status of the tap */
  if (int_status & BMI3_INT_STATUS_TAP) {
    ESP_LOGI(TAG, "Tap interrupt is generated");
    rslt = bmi323_get_regs(BMI3_REG_FEATURE_EVENT_EXT, data, 2, dev);

    if (data[0] & BMI3_TAP_DET_STATUS_SINGLE) {
      ESP_LOGI(TAG, "Single tap asserted");
    }

    if (data[0] & BMI3_TAP_DET_STATUS_DOUBLE) {
      ESP_LOGI(TAG, "Double tap asserted");
    }

    if (data[0] & BMI3_TAP_DET_STATUS_TRIPLE) {
      ESP_LOGI(TAG, "Triple tap asserted");
    }
    return data[0];
  }
  return 0;
}

/*!
 * @brief This internal API is used to set configurations for tap interrupt.
 */
static int8_t bmi3_set_feature_config(struct bmi3_dev *dev) {
  /* Status of API are returned to this variable. */
  int8_t rslt;

  /* Structure to define the type of sensor and its configurations. */
  struct bmi3_sens_config config[2] = {{0}};

  /* Configure the type of feature. */
  config[0].type = BMI323_ACCEL;
  config[1].type = BMI323_TAP;

  /* Get default configurations for the type of feature selected. */
  rslt = bmi323_get_sensor_config(config, 2, dev);
  bmi3_error_codes_print_result("Get sensor config", rslt);

  if (rslt == BMI323_OK) {
    /* Enable accel by selecting the mode. */
    config[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

    /* Set tap configuration settings. */

    /* Accelerometer sensing axis selection for tap detection.
     * Value    Name          Description
     * 00    axis_x     Use x-axis for tap detection
     * 01    axis_y     Use y-axis for tap detection
     * 10    axis_z     Use z-axis for tap detection
     * 11   reserved    Use z-axis for tap detection
     */
    config[1].cfg.tap.axis_sel = 1;

    /* Set new configurations. */
    rslt = bmi323_set_sensor_config(config, 2, dev);
    bmi3_error_codes_print_result("Set sensor config", rslt);
  }

  return rslt;
}
