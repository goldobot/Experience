// Driver pour le module PCA9685

#include "pca9685.h"

// Read one single-byte register from the slave. Provide the register's address as argument.
uint8_t pca_read (I2C_HandleTypeDef *hi2c, uint16_t reg)
{
    uint8_t val;
    HAL_StatusTypeDef stat = HAL_I2C_Mem_Read(hi2c, PCA9685_ADDR, reg, 1, &val, 1, 100000000);    // last parameter is timeout, no idea if it's long enough
    if (stat == HAL_OK)
    {
        return val;
    }
    return 0xAA; // may be used to help identify transmission errors.
}

// Write one single-byte register to slave. Provide the register's address and the data byte as arguments
uint8_t pca_write (I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t val)
{
    HAL_StatusTypeDef stat = HAL_I2C_Mem_Write(hi2c, PCA9685_ADDR, reg, 1, &val, 1, 100000000);    // last parameter is timeout, no idea if it's long enough
        if (stat == HAL_OK)
            return 0;
        else
            return 1;
}

// Write multiple registers to slave (address auto-increment). Provide the first register's address and the data bytes as arguments
// Note : autoincrement is disabled by default.

uint8_t pca_writebuff (I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t* val, uint16_t count)
{
    HAL_StatusTypeDef stat = HAL_I2C_Mem_Write(hi2c, PCA9685_ADDR, reg, 1, val, count, 100000000);    // last parameter is timeout, no idea if it's long enough
        if (stat == HAL_OK)
            return 0;
        else
            return 1;
}

// count is still in bytes !
uint8_t pca_writebuff16b (I2C_HandleTypeDef *hi2c, uint16_t reg, uint16_t* val, uint16_t count)
{
    HAL_StatusTypeDef stat = HAL_I2C_Mem_Write(hi2c, PCA9685_ADDR, reg, 1, (uint8_t*) val, count, 100000000);    // last parameter is timeout, no idea if it's long enough
        if (stat == HAL_OK)
            return 0;
        else
            return 1;
}




void pca_init (I2C_HandleTypeDef *hi2c)
{
    // Set prescaler for 50 Hz :
    pca_write (hi2c, PCA9685_PRESCALE, 0x79);
    // Enable address auto-increment, leave sleep mode, disable "all call" address
    pca_write (hi2c, PCA9685_MODE1_REG, 0x20);
    // Wait for oscillator to stabilize (0.5 ms)
    HAL_Delay (1);
}
