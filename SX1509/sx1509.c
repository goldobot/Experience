/*
 * sx1509.c
 *
 *  Created on: May 27, 2019
 *      Author: Nefastor
 */

#include "sx1509.h"

// Read one single-byte register from the slave. Provide the register's address as argument.
uint8_t sxread (I2C_HandleTypeDef *hi2c, uint16_t reg)
{
    uint8_t val;
    HAL_StatusTypeDef stat = HAL_I2C_Mem_Read(hi2c, SX1509_ADDR, reg, 1, &val, 1, 100000000);    // last parameter is timeout, no idea if it's long enough
    if (stat == HAL_OK)
    {
        return val;
    }
    return 0xAA; // may be used to help identify transmission errors.
}

// Write one single-byte register to slave. Provide the register's address and the data byte as arguments
uint8_t sxwrite (I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t val)
{
    HAL_StatusTypeDef stat = HAL_I2C_Mem_Write(hi2c, SX1509_ADDR, reg, 1, &val, 1, 100000000);    // last parameter is timeout, no idea if it's long enough
        if (stat == HAL_OK)
            return 0;
        else
            return 1;
}


// Vérification de présence du chip
// Retourne 0 si tout baigne.
// Fonction utilisée lors de tests en phase de dev, pas utile.
/*
uint8_t sxok (I2C_HandleTypeDef *hi2c)
{
    uint8_t r1;
    uint8_t rv = 0;
    r1 = sxread (hi2c, 0x0A); // Register 0x0A should be 0x00 at reset
    if (r1 == 0xAA)
        rv &= 0x01;
    r1 = sxread (hi2c, 0x0F); // Register 0x0F should be 0xFF at reset
    if (r1 == 0xAA)
        rv &= 0x02;
    return rv;
}*/


// Init des broches du SX1509 en tant que sorties
void sxinit (I2C_HandleTypeDef *hi2c)
{
    // Disable input buffer (RegInputDisable)
    sxwrite (hi2c, 0x00, 0xFF);   // port B
    sxwrite (hi2c, 0x01, 0xFF);   // port A
    // Disable pull-up (RegPullUp) => Disabled by default
    // Enable open drain (RegOpenDrain) => Default is push-pull
    // sxwrite (hi2c, 0x0A, 0xFF);   // port B
    // sxwrite (hi2c, 0x0B, 0xFF);   // port A
    // Set direction to output (RegDir) – by default all pins are inputs
    sxwrite (hi2c, 0x0E, 0x00);   // port B
    sxwrite (hi2c, 0x0F, 0x00);   // port A
    // Enable oscillator (RegClock)
    sxwrite (hi2c, 0x1E, 0x50);   // internal clock, clock pin as output, set to zero 0b0101_0000
    // Configure LED driver clock and mode if relevant (RegMisc)
    sxwrite (hi2c, 0x1F, 0x10);     // enable LED driver clock, 0b0001_0000
    // Enable LED driver operation (RegLEDDriverEnable)
    sxwrite (hi2c, 0x20, 0xFF);   // port B
    sxwrite (hi2c, 0x21, 0xFF);   // port A
    // Configure LED driver parameters (RegTOn, RegIOn, RegOff, RegTRise, RegTFall)
    // Set RegData bit low => LED driver started
}


// Envoi des valeurs aux broches de sortie du module. Port A : broches 0 à 7 (bandeaux de LED du décors). Port B : inutilisé pour le moment
void sxout (I2C_HandleTypeDef *hi2c, uint8_t portA, uint8_t portB)
{
    sxwrite (hi2c, 0x10, portB);   // port B
    sxwrite (hi2c, 0x11, portA);   // port A
}

