/*
 * comms.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Max
 */

#include "comms.h"

void comms_send_data(SPI_HandleTypeDef *hspi, int16_t score, int16_t temp,
    GPIO_TypeDef *ready_port, uint16_t ready_pin) {

  HAL_GPIO_WritePin(ready_port, ready_pin, GPIO_PIN_SET);

  uint8_t spi_payload[4] = { (uint8_t) (score >> 8), (uint8_t) (score & 0xFF),
      (uint8_t) (temp >> 8), (uint8_t) (temp & 0xFF) };

  uint8_t rx_placeholder[sizeof(spi_payload)] = { 0 };

  HAL_SPI_TransmitReceive(hspi, spi_payload, rx_placeholder,
      sizeof(spi_payload), 100);

  HAL_GPIO_WritePin(ready_port, ready_pin, GPIO_PIN_RESET);
}
