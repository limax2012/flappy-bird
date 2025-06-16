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

void comms_send_data(SPI_HandleTypeDef *hspi, int16_t score, int16_t temp,
    GPIO_TypeDef *ready_port, uint16_t ready_pin);

#endif /* INC_COMMS_H_ */
