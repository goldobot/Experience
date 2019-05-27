// Driver pour le module PCA9685

#include "pca9685.h"

// Table des positions des servos de la figurine (sera envoyée par DMA)
uint16_t servos[16]; // 8 servos, pour chacun, Ton et Toff

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

// Transfert par DMA, limité à 8 canaux (l'Experience n'en utilise pas plus)
// Le buffer passé en argument doit avoir une taille de 16 mots de 16 bits
void pca_dma_start (I2C_HandleTypeDef *hi2c)
{
    // arguments :
    // 1 : interface I2C
    // 2 : adresse de l'esclave
    // 3 : adresse du registre dans l'esclave : premier registre du premier canal du PCA9685
    // 4 : taille de l'adresse de registre
    // 5 : pointeur sur les données à envoyer
    // 6 : nombre d'octets de données à envoyer
    HAL_StatusTypeDef rv = HAL_I2C_Mem_Write_DMA(hi2c, PCA9685_ADDR, PCA9685_LED8_ON_L, 1, (uint8_t*) servos, 32);
    //if (rv == HAL_OK)
    //    return;

    // Different type of transfer
    uint8_t dat[] = {0x26, 0, 0, 150, 0};
    //HAL_I2C_Master_Transmit_DMA(hi2c, PCA9685_ADDR, dat, 5);
}

// Surcharge de la callback DMA I²C
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // Cette callback est appelée en fin de transfert DMA. Elle va servir à relancer le transfert, après avoir
    // entrelacé d'autres transferts I²C


        // Tout d'abord, il semble necessaire de generer une condition STOP (le transfert DMA ne le fait pas) :
            // https://community.st.com/s/question/0D50X00009XkeAqSAJ/difference-between-hali2cmemwrite-and-hali2cmemwritedma

        //I2C_MasterTransmit_BTF(hi2c);

        // Ensuite, on peut relancer le transfert :
        HAL_I2C_Mem_Write_DMA(hi2c, PCA9685_ADDR, PCA9685_LED8_ON_L, 1, (uint8_t*) servos, 32);




}
