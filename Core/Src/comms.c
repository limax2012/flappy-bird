/*
 * comms.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Max
 */

#include "comms.h"

void comms_send_score_temp(SPI_HandleTypeDef *hspi, int score, int32_t temp) {
  int16_t transfer_score = (int16_t) score;
  int16_t transfer_temp = (int16_t) temp;

  uint8_t spi_payload[4] = { (uint8_t) (transfer_score >> 8),
      (uint8_t) (transfer_score & 0xFF), (uint8_t) (transfer_temp >> 8),
      (uint8_t) (transfer_temp & 0xFF) };

  uint8_t rx_placeholder[sizeof(spi_payload)] = { 0 };

  HAL_SPI_TransmitReceive(hspi, spi_payload, rx_placeholder,
      sizeof(spi_payload), 100);
}
