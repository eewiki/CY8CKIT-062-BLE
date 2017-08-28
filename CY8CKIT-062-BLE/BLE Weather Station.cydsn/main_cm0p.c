/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "I2C_1.h"

#define I2C_TIMEOUT 100

#define SI7005_ADDR 0x40
#define SI7005_DEV_ID 0x50

/* SI7005 Registers */
#define SI7005_REG_STATUS         0x00
#define SI7005_REG_DATA           0x01
#define SI7005_REG_CONFIG         0x03
#define SI7005_REG_ID             0x11

/* SI7005 Status Register */
#define SI7005_STATUS_NOT_READY   0x01

/* SI7005 Config Register */
#define SI7005_CONFIG_START       0x01
#define SI7005_CONFIG_HEAT        0x02
#define SI7005_CONFIG_HUMIDITY    0x00
#define SI7005_CONFIG_HUMIDITY_START    0x01
#define SI7005_CONFIG_TEMPERATURE 0x10
#define SI7005_CONFIG_TEMPERATURE_START 0x11
#define SI7005_CONFIG_FAST        0x20

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    
    I2C_1_Start();

    volatile uint8_t tmp0 = 0;
    volatile uint8_t si7005_connected = 0;
    
    I2C_1_MasterSendStart(SI7005_ADDR, CY_SCB_I2C_WRITE_XFER, I2C_TIMEOUT);
    I2C_1_MasterWriteByte(SI7005_REG_ID, I2C_TIMEOUT);
    I2C_1_MasterSendReStart(SI7005_ADDR, CY_SCB_I2C_READ_XFER, I2C_TIMEOUT);    
    I2C_1_MasterReadByte(CY_SCB_I2C_NAK, &tmp0, I2C_TIMEOUT);
    I2C_1_MasterSendStop(I2C_TIMEOUT);
       
    if ( tmp0 == SI7005_DEV_ID )
        si7005_connected=1;    

    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
