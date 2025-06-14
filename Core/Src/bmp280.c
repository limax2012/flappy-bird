/*
 * bmp280.c
 *
 *  Created on: Jun 13, 2025
 *      Author: Max
 */

#include "bmp280.h"

static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;

void bmp280_calib(I2C_HandleTypeDef *hi2c, osMutexId_t mutex_id) {
  static const uint8_t calib_start_reg = 0x88;
  uint8_t calib_buf[6];

  if (osMutexAcquire(mutex_id, osWaitForever) == osOK) {
    HAL_I2C_Mem_Read(hi2c, BMP280_I2C_ADDR << 1, calib_start_reg, 1, calib_buf,
        6, 100);

    osMutexRelease(mutex_id);
  }

  dig_T1 = (uint16_t) (calib_buf[1] << 8 | calib_buf[0]);
  dig_T2 = (int16_t) (calib_buf[3] << 8 | calib_buf[2]);
  dig_T3 = (int16_t) (calib_buf[5] << 8 | calib_buf[4]);
}

int32_t bmp280_get_temp(I2C_HandleTypeDef *hi2c, osMutexId_t mutex_id) {
  static const uint8_t ctrl_meas_write[2] = { 0xF4, 0x21 }; // osrs_t = x1, osrs_p = skip, mode = forced
  static const uint8_t temp_start_reg = 0xFA;
  uint8_t temp_reg_data[3];

  if (osMutexAcquire(mutex_id, osWaitForever) == osOK) {
    HAL_I2C_Master_Transmit(hi2c, BMP280_I2C_ADDR << 1, ctrl_meas_write, 2,
        100);
    HAL_Delay(5);

    HAL_I2C_Master_Transmit(hi2c, BMP280_I2C_ADDR << 1, &temp_start_reg, 1,
        100);
    HAL_I2C_Master_Receive(hi2c, BMP280_I2C_ADDR << 1, temp_reg_data, 3, 100);

    osMutexRelease(mutex_id);
  }

  int32_t raw_temp = ((int32_t) temp_reg_data[0] << 12)
      | ((int32_t) temp_reg_data[1] << 4) | (temp_reg_data[2] >> 4);

  int32_t var1 = ((((raw_temp >> 3) - ((int32_t) dig_T1 << 1)))
      * ((int32_t) dig_T2)) >> 11;
  int32_t var2 = (((((raw_temp >> 4) - ((int32_t) dig_T1))
      * ((raw_temp >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t) dig_T3))
      >> 14;

  int32_t t_fine = var1 + var2;
  int32_t T = (t_fine * 5 + 128) >> 8; // T in 0.01°C

  return T;
}
