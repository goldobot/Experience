/*
 * sx1509.h
 *
 *  Created on: May 27, 2019
 *      Author: Nefastor
 */

#ifndef _SX1509_H_
#define _SX1509_H_

#include "stm32f3xx_hal.h"
#include "sx1509_registers.h"

// Note : addresse d'esclave codée en dur pour le moment : 0x3E
// #define SX1509_ADDR 0x3E
// Note 2 : la HAL STM32 requiert de shifter l'adresse pour prendre en compte le bit R/W :
#define SX1509_ADDR 0x7C

uint8_t sxread (I2C_HandleTypeDef *hi2c, uint16_t reg);
uint8_t sxwrite (I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t val);
uint8_t sxok (I2C_HandleTypeDef *hi2c);
void sxinit (I2C_HandleTypeDef *hi2c);
void sxout (I2C_HandleTypeDef *hi2c, uint8_t portA, uint8_t portB);

#endif /* _SX1509_H_ */
