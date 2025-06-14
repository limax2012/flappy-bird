/*
 * temp.h
 *
 *  Created on: Jun 13, 2025
 *      Author: Max
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

#define BMP280_I2C_ADDR 0x76

void bmp280_calib(I2C_HandleTypeDef *hi2c, osMutexId_t mutex_id);
int32_t bmp280_get_temp(I2C_HandleTypeDef *hi2c, osMutexId_t mutex_id);

#endif /* INC_BMP280_H_ */
