#ifndef __PCA9685_H__
#define __PCA9685_H__

#include "stm32f3xx_hal.h"

#define PCA9685_ADDR    0x80
// The board we're using has pull-downs on all address lines, hence the 0x40.
// The STM32 HAL requires a left bit shift to account for the R/W bit.

#define PCA9685_MODE1_REG       0x00
#define PCA9685_MODE2_REG       0x01
#define PCA9685_LED0_ON_L       0x06
#define PCA9685_LED1_ON_L       0x0A

#define PCA9685_LED2_ON_L       0x0E
#define PCA9685_LED3_ON_L       0x12
#define PCA9685_LED4_ON_L       0x16
#define PCA9685_LED5_ON_L       0x1A
#define PCA9685_LED6_ON_L       0x1E
#define PCA9685_LED7_ON_L       0x22
#define PCA9685_LED8_ON_L       0x26
#define PCA9685_LED9_ON_L       0x2A
#define PCA9685_LED10_ON_L       0x2E
#define PCA9685_LED11_ON_L       0x32
#define PCA9685_LED12_ON_L       0x36
#define PCA9685_LED13_ON_L       0x3A
#define PCA9685_LED14_ON_L       0x3E
#define PCA9685_LED15_ON_L       0x42

#define PCA9685_ALL_LED_ON_L    0xfa
#define PCA9685_PRESCALE        0xfe

#define PCA9685_SLEEP_BIT       0x10
#define PCA9685_EXTCLK_BIT      0x40
#define PCA9685_RESTART_BIT     0x80

// Variables exportées

// Table des positions des servos de la figurine (sera envoyée par DMA)
extern uint16_t servos[16]; // 8 servos, pour chacun, Ton et Toff

// Fonctions exportées


uint8_t pca_read (I2C_HandleTypeDef *hi2c, uint16_t reg);
uint8_t pca_write (I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t val);
uint8_t pca_writebuff (I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t* val, uint16_t count);
uint8_t pca_writebuff16b (I2C_HandleTypeDef *hi2c, uint16_t reg, uint16_t* val, uint16_t count);
void pca_init (I2C_HandleTypeDef *hi2c);
void pca_dma_start (I2C_HandleTypeDef *hi2c);

#endif __PCA9685_H__
