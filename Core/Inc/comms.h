/*
 * comms.h
 *
 *  Created on: Jun 15, 2025
 *      Author: Max
 */

#ifndef INC_COMMS_H_
#define INC_COMMS_H_

#include <stdint.h>

#include "stm32f1xx_hal.h"

void comms_send_score_temp(SPI_HandleTypeDef *hspi, int score, int32_t temp);

#endif /* INC_COMMS_H_ */
