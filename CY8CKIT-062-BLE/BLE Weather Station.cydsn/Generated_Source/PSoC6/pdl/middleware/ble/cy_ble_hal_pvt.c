/***************************************************************************//**
* \file cy_ble_hal_pvt.c
* \version 1.0
*
* \brief
*  This file contains the source code for the HAL section of the BLE component
*
********************************************************************************
* \copyright
* Copyright 2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_ble_event_handler.h"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "cy_ble_hal_pvt.h"

#if (CY_BLE_HOST_CONTR_CORE)

#if (CY_BLE_STACK_MODE_HOST_UART)


/*******************************************************************************
* Function Name: Cy_BLE_HAL_HOST_UART_Start
****************************************************************************//**
*
*  Enables the platform UART TX and RX interrupts and then enables the UART
*  component.
*
* \return
*  None
*
*******************************************************************************/
void Cy_BLE_HAL_HOST_UART_Start(void)
{
    /* Setup ISR */
    Cy_SysInt_Init(cy_ble_configPtr->uartHostIsrConfig, &Cy_BLE_HAL_HOST_UART_Interrupt);
    NVIC_EnableIRQ((IRQn_Type)cy_ble_configPtr->uartHostIsrConfig->intrSrc);

    (void)Cy_SCB_UART_Init(cy_ble_configPtr->uartHostHw, cy_ble_configPtr->uartHostConfig, NULL);
    /* Configure interrupt sources. */
    Cy_SCB_SetRxInterruptMask(cy_ble_configPtr->uartHostHw, CY_SCB_RX_INTR_NOT_EMPTY | CY_SCB_RX_INTR_OVERFLOW |
                              CY_SCB_RX_INTR_UNDERFLOW);
    Cy_SCB_SetTxInterruptMask(cy_ble_configPtr->uartHostHw, CY_SCB_TX_INTR_OVERFLOW | CY_SCB_TX_INTR_UART_DONE);

    Cy_SCB_UART_Enable(cy_ble_configPtr->uartHostHw);
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_HOST_UART_Stop
****************************************************************************//**
*
*  Disables the UART, clears all pending interrupts and disables the UART TX
*  and RX interrupts. This will also empty out the FIFOs.
*
* \return
*  None
*
*******************************************************************************/
void Cy_BLE_HAL_HOST_UART_Stop(void)
{
    /* Stop interrupt and UART */
    NVIC_DisableIRQ((IRQn_Type)cy_ble_configPtr->uartHostIsrConfig->intrSrc);

    Cy_SCB_UART_Disable(cy_ble_configPtr->uartHostHw, NULL);
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_HOST_UART_Transmit
****************************************************************************//**
*
*  Sends the data to UART TX FIFO. The function handles data length up to the
*  supported TX FIFO length of the UART hardware module.
*
*  \param data: Pointer to the data to send through the UART
*  \param length: the length of data to transmit in bytes
*
* \return
*  None
*
*******************************************************************************/
void Cy_BLE_HAL_HOST_UART_Transmit(const uint8_t *dataBuf,
                                   uint8_t length)
{
    Cy_SCB_UART_PutArrayBlocking(cy_ble_configPtr->uartHostHw, (uint8_t*)dataBuf, (uint32_t)length);
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_HOST_UART_IsrEnable
****************************************************************************//**
*
*   Enables the UART interrupt to the interrupt controller.
*   Do not call this function unless Cy_BLE_HAL_HOST_UART_Start() has been called or the
*   functionality of the Cy_SysInt_Init() function, which sets the vector and the
*   priority, has been called.
*
*******************************************************************************/
void Cy_BLE_HAL_HOST_UART_IsrEnable(void)
{
    NVIC_EnableIRQ((IRQn_Type)cy_ble_configPtr->uartHostIsrConfig->intrSrc);
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_HOST_UART_IsrDisable
****************************************************************************//**
*
*   Disables the UART Interrupt in the interrupt controller.
*
*******************************************************************************/
void Cy_BLE_HAL_HOST_UART_IsrDisable(void)
{
    NVIC_DisableIRQ((IRQn_Type)cy_ble_configPtr->uartHostIsrConfig->intrSrc);
}

#endif /* (CY_BLE_STACK_MODE_HOST_UART) */


#if (CY_BLE_CONFIG_STACK_MODE_CONTR_UART)


/*******************************************************************************
* Function Name: Cy_BLE_HAL_UART_Interrupt
****************************************************************************//**
*
*  Handles the Interrupt Service Routine for the UART.
*
* \return
*  None
*
*******************************************************************************/
void Cy_BLE_HAL_UART_Interrupt(void)
{
    uint8_t uartTxStatus = CY_BLE_INTR_TX_SUCCESS;
    uint32_t srcInterrupt;

    uint8_t length = 0u;
    uint8_t srcCount = 0u;
    uint8_t uartRxStatus = CY_BLE_INTR_RX_SUCCESS;
    uint8_t receivedData;

    /* Evaluate RX interrupt event */
    srcInterrupt = Cy_SCB_GetRxInterruptStatusMasked(cy_ble_configPtr->uartHw);

    if(0u != (srcInterrupt & CY_SCB_RX_INTR_NOT_EMPTY))
    {
        if(0u != (srcInterrupt & CY_SCB_RX_INTR_UART_PARITY_ERROR))
        {
            uartRxStatus |= CY_BLE_INTR_RX_PARITY_ERROR;
        }
        if(0u != (srcInterrupt & CY_SCB_RX_INTR_UART_FRAME_ERROR))
        {
            uartRxStatus |= CY_BLE_INTR_RX_FRAME_ERROR;
        }
        if(0u != (srcInterrupt & CY_SCB_RX_INTR_OVERFLOW))
        {
            uartRxStatus |= CY_BLE_INTR_RX_OVERFLOW;
        }
        if(uartRxStatus == CY_BLE_INTR_RX_SUCCESS)
        {
            length = (uint8_t)Cy_SCB_GetNumInRxFifo(cy_ble_configPtr->uartHw);
            for(srcCount = 0u; srcCount < length; srcCount++)
            {
                receivedData = (uint8_t)Cy_SCB_ReadRxFifo(cy_ble_configPtr->uartHw);
                Cy_BLE_HAL_UartRxDataHandler(receivedData);
            }
        }
        else
        {
            Cy_SCB_ClearRxFifo(cy_ble_configPtr->uartHw);
        }
        Cy_SCB_ClearRxInterrupt(cy_ble_configPtr->uartHw, srcInterrupt);
    }
    else
    {
        /* No RX interrupt. Do nothing. */
    }

    /* Evaluate TX interrupt event in sequence */
    srcInterrupt = Cy_SCB_GetTxInterruptStatusMasked(cy_ble_configPtr->uartHw);

    /* Stack manager TX UART complete */
    if(0u != (srcInterrupt & CY_SCB_TX_INTR_UART_DONE))
    {
        if(0u != (srcInterrupt & CY_SCB_TX_INTR_OVERFLOW))
        {
            /*Stack manager TX UART error */
            uartTxStatus |= CY_BLE_INTR_TX_OVERFLOW;
        }
        Cy_BLE_HAL_UartTxCompltHandler();
        Cy_SCB_ClearTxInterrupt(cy_ble_configPtr->uartHw, srcInterrupt);
    }
    else
    {
        /* No TX interrupt. Do nothing. */
    }
}

/*******************************************************************************
* Function Name: Cy_BLE_HAL_UART_Start
****************************************************************************//**
*
*  Enables the platform UART TX and RX interrupts and then enables the UART
*  component.
*
* \return
*  None
*
*******************************************************************************/
void Cy_BLE_HAL_UART_Start(void)
{
    /* Setup ISR */
    Cy_SysInt_Init(cy_ble_configPtr->uartIsrConfig, &Cy_BLE_HAL_UART_Interrupt);
    NVIC_EnableIRQ((IRQn_Type)cy_ble_configPtr->uartIsrConfig->intrSrc);

    (void)Cy_SCB_UART_Init(cy_ble_configPtr->uartHw, cy_ble_configPtr->uartConfig, NULL);
    /* Configure interrupt sources. */
    Cy_SCB_SetRxInterruptMask(cy_ble_configPtr->uartHw,
                              CY_SCB_RX_INTR_NOT_EMPTY | CY_SCB_RX_INTR_OVERFLOW | CY_SCB_RX_INTR_UNDERFLOW);
    Cy_SCB_SetTxInterruptMask(cy_ble_configPtr->uartHw, CY_SCB_TX_INTR_OVERFLOW | CY_SCB_TX_INTR_UART_DONE);

    Cy_SCB_UART_Enable(cy_ble_configPtr->uartHw);
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_UART_Stop
****************************************************************************//**
*
*  Disables the UART, clears all pending interrupts and disables the UART TX
*  and RX interrupts. This will also empty out the FIFOs.
*
* \return
*  None
*
*******************************************************************************/
void Cy_BLE_HAL_UART_Stop(void)
{
    /* Stop interrupt and UART */
    NVIC_DisableIRQ((IRQn_Type)cy_ble_configPtr->uartIsrConfig->intrSrc);

    Cy_SCB_UART_Disable(cy_ble_configPtr->uartHw, NULL);
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_UART_Transmit
****************************************************************************//**
*
*  Sends the data to UART TX FIFO. The function handles data length up to the
*  supported TX FIFO length of the UART hardware module.
*
*  \param data: Pointer to the data to send through the UART
*  \param length: the length of data to transmit in bytes
*
* \return
*  None
*
*******************************************************************************/
void Cy_BLE_HAL_UART_Transmit(uint8_t *dataBuf,
                              uint8_t length)
{
    Cy_SCB_UART_PutArrayBlocking(cy_ble_configPtr->uartHw, dataBuf, (uint32_t)length);
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_UART_IsrEnable
****************************************************************************//**
*
*   Enables the UART interrupt to the interrupt controller.
*   Do not call this function unless Cy_BLE_HAL_UART_Start() has been called or the
*   functionality of the Cy_SysInt_Init() function, which sets the vector and the
*   priority, has been called.
*
*******************************************************************************/
void Cy_BLE_HAL_UART_IsrEnable(void)
{
    NVIC_EnableIRQ((IRQn_Type)cy_ble_configPtr->uartIsrConfig->intrSrc);
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_UART_IsrDisable
****************************************************************************//**
*
*   Disables the UART Interrupt in the interrupt controller.
*
*******************************************************************************/
void Cy_BLE_HAL_UART_IsrDisable(void)
{
    NVIC_DisableIRQ((IRQn_Type)cy_ble_configPtr->uartIsrConfig->intrSrc);
}

#endif /* (CY_BLE_CONFIG_STACK_MODE_CONTR_UART) */

#if (CY_BLE_MODE_PROFILE)


/*******************************************************************************
* Function Name: Cy_BLE_HAL_NvramWrite
****************************************************************************//**
*
*  This function writes the data to the NVRAM store. It will check the
*  appropriate alignment of a start address and also perform an address range
*  check based on the length before performing the write operation.
*  This function performs memory compare and writes only row where there are new
*  data to write.
*
*  \param buffer:   Pointer to the buffer containing the data to be stored.
*  \param varFlash: Pointer to the array or variable in the flash.
*  \param length:    The length of the data in bytes.
*
* \return
*  CY_BLE_SUCCESS                     A successful write
*  CY_BLE_ERROR_INVALID_PARAMETER     At least one of the input parameters is invalid
*  CY_BLE_ERROR_FLASH_WRITE           Error in flash Write
*
*******************************************************************************/
cy_en_ble_api_result_t Cy_BLE_HAL_NvramWrite(const uint8_t buffer[],
                                             const uint8_t varFlash[],
                                             uint32_t length)
{
    cy_en_ble_api_result_t bleReturn = CY_BLE_SUCCESS;
    cy_en_flashdrv_status_t rc = CY_FLASH_DRV_SUCCESS;

    uint32_t writeBuffer[CY_FLASH_SIZEOF_ROW / sizeof(uint32_t)];
    uint32_t rowId;
    uint32_t dstIndex;
    uint32_t srcIndex = 0u;
    uint32_t eeOffset;
    uint32_t byteOffset;
    uint32_t rowsNotEqual;
    uint8_t *writeBufferPointer;  

    eeOffset = (uint32_t)varFlash;
    writeBufferPointer = (uint8_t*)writeBuffer;

    /* Make sure, that varFlash[] points to Flash or WFlash */
    if(((eeOffset >= CY_FLASH_BASE) &&
        ((eeOffset + length) <= (CY_FLASH_BASE + CY_FLASH_SIZE))) ||
       ((eeOffset >= CY_WFLASH_BASE) &&
        ((eeOffset + length) <= (CY_WFLASH_BASE + CY_WFLASH_SIZE))))
    {
        cy_en_syspm_simo_buck_voltage1_t simoVoltage = CY_SYSPM_SIMO_BUCK_OUT1_VOLTAGE_1_1V;
        cy_en_syspm_ldo_voltage_t ldoVoltage = CY_SYSPM_LDO_VOLTAGE_1_1V;
        
        if(Cy_SysPm_SimoBuckIsEnabled())
        {
            simoVoltage = Cy_SysPm_SimoBuckGetVoltage1();
            if(simoVoltage == CY_SYSPM_SIMO_BUCK_OUT1_VOLTAGE_0_9V)
            {   /* Increase core voltage for write to flash operation */
                Cy_SysPm_SimoBuckSetVoltage1(CY_SYSPM_SIMO_BUCK_OUT1_VOLTAGE_1_1V);
            }
        }
        if(Cy_SysPm_LdoIsEnabled())
        {
            ldoVoltage = Cy_SysPm_LdoGetVoltage();
            if(ldoVoltage == CY_SYSPM_LDO_VOLTAGE_0_9V)
            {   /* Increase core voltage for write to flash operation */
                 Cy_SysPm_LdoSetVoltage(CY_SYSPM_LDO_VOLTAGE_1_1V);
            }
        }
        
        eeOffset -= CY_FLASH_BASE;
        rowId = eeOffset / CY_FLASH_SIZEOF_ROW;
        byteOffset = CY_FLASH_SIZEOF_ROW * rowId;

        while((srcIndex < length) && (rc == CY_FLASH_DRV_SUCCESS))
        {
            rowsNotEqual = 0u;
            /* Copy data to the write buffer either from the source buffer or from the flash */
            for(dstIndex = 0u; dstIndex < CY_FLASH_SIZEOF_ROW; dstIndex++)
            {
                if((byteOffset >= eeOffset) && (srcIndex < length))
                {
                    writeBufferPointer[dstIndex] = buffer[srcIndex];
                    /* Detect that row programming is required */
                    if((rowsNotEqual == 0u) && (CY_GET_REG8(CY_FLASH_BASE + byteOffset) != buffer[srcIndex]))
                    {
                        rowsNotEqual = 1u;
                    }
                    srcIndex++;
                }
                else
                {
                    writeBufferPointer[dstIndex] = CY_GET_REG8(CY_FLASH_BASE + byteOffset);
                }
                byteOffset++;
            }

            if(rowsNotEqual != 0u)
            {
                /* Write flash row */
                rc = Cy_Flash_WriteRow((rowId * CY_FLASH_SIZEOF_ROW) + CY_FLASH_BASE, writeBuffer);
            }

            /* Go to the next row */
            rowId++;
        }
        if((Cy_SysPm_SimoBuckIsEnabled()) && (simoVoltage == CY_SYSPM_SIMO_BUCK_OUT1_VOLTAGE_0_9V))
        {   /* Return core voltage */
            Cy_SysPm_SimoBuckSetVoltage1(CY_SYSPM_SIMO_BUCK_OUT1_VOLTAGE_0_9V);
        }
        if((Cy_SysPm_LdoIsEnabled()) && (ldoVoltage == CY_SYSPM_LDO_VOLTAGE_0_9V))
        {   /* Return core voltage */
            Cy_SysPm_LdoSetVoltage(CY_SYSPM_LDO_VOLTAGE_0_9V);
        }
    }
    else
    {
        rc = CY_FLASH_DRV_INVALID_INPUT_PARAMETERS;
    }

    /* Return BLE error code */
    switch(rc)
    {
        case CY_FLASH_DRV_SUCCESS:
            bleReturn = CY_BLE_SUCCESS;
            break;

        case CY_FLASH_DRV_INVALID_INPUT_PARAMETERS:
        case CY_FLASH_DRV_INVALID_FLASH_ADDR:
            bleReturn = CY_BLE_ERROR_INVALID_PARAMETER;
            break;

        default:
            bleReturn = CY_BLE_ERROR_FLASH_WRITE;
            break;
    }

    return(bleReturn);
}

#endif /* CY_BLE_MODE_PROFILE */


/*******************************************************************************
* Function Name: Cy_BLE_HAL_DelayUs
****************************************************************************//**
*
*  Interface to Cy_SysLib_DelayUs function.
*
*******************************************************************************/
void Cy_BLE_HAL_DelayUs(uint16_t delayVal)
{
    Cy_SysLib_DelayUs(delayVal);
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_DelayMs
****************************************************************************//**
*
*  Interface to Cy_SysLib_Delay function.
*
*******************************************************************************/
void Cy_BLE_HAL_DelayMs(uint32_t delayVal)
{
    Cy_SysLib_Delay(delayVal);
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_EnableGlobalInterrupts
****************************************************************************//**
*
*  This function enables all interrupt used by the component.
*
*******************************************************************************/
void Cy_BLE_HAL_EnableGlobalInterrupts(void)
{
#if (CY_BLE_STACK_MODE_HOST_UART)
    Cy_BLE_HAL_HOST_UART_IsrEnable();
#endif /* (CY_BLE_STACK_MODE_HOST_UART) */

#if (CY_BLE_CONFIG_STACK_MODE_CONTR_UART)
    Cy_BLE_HAL_UART_IsrEnable();
#endif /* (CY_BLE_CONFIG_STACK_MODE_CONTR_UART) */

/* Enable IPC interrupt */
#if (CY_BLE_STACK_MODE_IPC)
    Cy_BLE_Ipc_Cypipe_Isr_Enable();
#endif  /* (CY_BLE_CONFIG_STACK_CONTR_CORE) */

#if (CY_BLE_CONFIG_STACK_CONTR_CORE)
    NVIC_EnableIRQ((IRQn_Type)cy_ble_configPtr->blessIsrConfig->intrSrc);
#endif  /* (CY_BLE_CONFIG_STACK_CONTR_CORE) */
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_DisableGlobalInterrupts
****************************************************************************//**
*
*  This disables all interrupt used by the component.
*
*******************************************************************************/
void Cy_BLE_HAL_DisableGlobalInterrupts(void)
{
#if (CY_BLE_STACK_MODE_HOST_UART)
    Cy_BLE_HAL_HOST_UART_IsrDisable();
#endif /* (CY_BLE_STACK_MODE_HOST_UART) */

#if (CY_BLE_CONFIG_STACK_MODE_CONTR_UART)
    Cy_BLE_HAL_UART_IsrDisable();
#endif /* (CY_BLE_CONFIG_STACK_MODE_CONTR_UART) */

/* Disable IPC interrupt */
#if (CY_BLE_STACK_MODE_IPC)
    Cy_BLE_Ipc_Cypipe_Isr_Disable();
#endif /* (CY_BLE_STACK_MODE_IPC) */

#if (CY_BLE_CONFIG_STACK_CONTR_CORE)
    NVIC_DisableIRQ((IRQn_Type)cy_ble_configPtr->blessIsrConfig->intrSrc);
#endif /* (CY_BLE_CONFIG_STACK_CONTR_CORE) */
}

/* Stack Interface to clock */

/*******************************************************************************
* Function Name: Cy_BLE_HAL_LfClkGetSource
****************************************************************************//**
*
*  Interface to Cy_SysClk_LfClkGetSource function.
*
*******************************************************************************/
cy_en_clklf_in_sources_t Cy_BLE_HAL_LfClkGetSource(void)
{
    return(Cy_SysClk_ClkLfGetSource());
}

/*******************************************************************************
* Function Name: Cy_BLE_HAL_ClkMeasurementCountersDone
****************************************************************************//**
*
*  Interface to Cy_SysClk_ClkMeasurementCountersDone function.
*
*******************************************************************************/
int32_t Cy_BLE_HAL_ClkMeasurementCountersDone(void)
{
    return(Cy_SysClk_ClkMeasurementCountersDone());
}

/*******************************************************************************
* Function Name: Cy_BLE_HAL_ClkMeasurementCountersGetClock2Freq
****************************************************************************//**
*
*  Interface to Cy_SysClk_ClkMeasurementCountersGetFreq function.
*
*******************************************************************************/
uint32_t Cy_BLE_HAL_ClkMeasurementCountersGetFreq(bool measuredClock,
                                                  uint32_t refClkFreq)
{
    return(Cy_SysClk_ClkMeasurementCountersGetFreq(measuredClock, refClkFreq));
}

/*******************************************************************************
* Function Name: Cy_BLE_HAL_StartClkMeasurementCounters
****************************************************************************//**
*
*  Interface to Cy_SysClk_StartClkMeasurementCounters function.
*
*******************************************************************************/
uint32_t Cy_BLE_HAL_StartClkMeasurementCounters(cy_en_meas_clks_t clock1,
                                                uint32_t count1,
                                                cy_en_meas_clks_t clock2)
{
    return(Cy_SysClk_StartClkMeasurementCounters(clock1, count1, clock2));
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_PiloTrim
****************************************************************************//**
*
*  Interface to Cy_SysClk_PiloTrim function.
*
*******************************************************************************/
int32_t Cy_BLE_HAL_PiloTrim(uint32_t piloFreq)
{
    return(Cy_SysClk_PiloTrim(piloFreq));
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_BlessStart
****************************************************************************//**
*
*  Starts Interrupt Controller.
*
* \return
*  None
*
*******************************************************************************/
void Cy_BLE_HAL_BlessStart(void)
{
    Cy_BLE_HAL_Init();

    /* Configures external power apmlifier outputs. */
    Cy_BLE_ConfigureExtPA(BLE_BLESS_EXT_PA_LNA_CTRL_ENABLE_EXT_PA_LNA_Msk |
                          (CY_BLE_CONFIG_EXT_PA_ENABLED ? BLE_BLESS_EXT_PA_LNA_CTRL_OUT_EN_DRIVE_VAL_Msk : 0u));

#if (CY_BLE_CONFIG_STACK_CONTR_CORE)
    /* Setup BLESS ISR */
    (void)Cy_SysInt_Init(cy_ble_configPtr->blessIsrConfig, &Cy_BLE_BlessInterrupt);
    NVIC_EnableIRQ((IRQn_Type)cy_ble_configPtr->blessIsrConfig->intrSrc);
#endif /* (CY_BLE_CONFIG_STACK_CONTR_CORE) */
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_GetIpBlockVersion
****************************************************************************//**
*
* This function returns the version of m0s8bless ip block.
*
* \return
* uint32_t bits:
* 7:0 - ip version ( 1 - BLE_ver1, 2 - BLE_ver2, 3 - BLE_ver3, 4 - BLE_ver3 (*A) )
* 31:8 - reserved for future usage
*
*******************************************************************************/
uint32_t Cy_BLE_HAL_GetIpBlockVersion(void)
{
    return(CY_BLE_M0S8BLESS_VERSION);
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_EnterCriticalSection
****************************************************************************//**
*
*  Interface to Cy_SysLib_EnterCriticalSection function.
*
*******************************************************************************/
uint32_t Cy_BLE_HAL_EnterCriticalSection(void)
{
    return(Cy_SysLib_EnterCriticalSection());
}

/*******************************************************************************
* Function Name: Cy_BLE_HAL_ExitCriticalSection
****************************************************************************//**
*
*  Interface to Cy_SysLib_ExitCriticalSection function.
*
*******************************************************************************/
void Cy_BLE_HAL_ExitCriticalSection(uint32_t interruptState)
{
    Cy_SysLib_ExitCriticalSection(interruptState);
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_SysPmSleep
****************************************************************************//**
*
*  Interface to Cy_SysPm_Sleep function.
*
*******************************************************************************/
cy_en_syspm_status_t Cy_BLE_HAL_SysPmSleep(cy_en_syspm_waitfor_t enWaitFor)
{
    return(Cy_SysPm_Sleep(enWaitFor));
}


/*******************************************************************************
* Function Name: Cy_BLE_HAL_SimoBuckSetVoltage2
****************************************************************************//**
*
*  Interface to Cy_SysPm_SimoBuckSetVoltage2 function with disable the 200 uS
*  delay after setting a higher voltage.
*
*******************************************************************************/
void Cy_BLE_HAL_SimoBuckSetVoltage2(cy_en_syspm_simo_buck_voltage2_t voltage)
{
    Cy_SysPm_SimoBuckSetVoltage2(voltage, false);
}


#if (CY_BLE_STACK_MODE_IPC)


/*******************************************************************************
* Function Name: Cy_BLE_Ipc_Cypipe_Isr_Enable
****************************************************************************//**
*
*   Enables the IPC pipe interrupt to the interrupt controller.
*
*******************************************************************************/
void Cy_BLE_Ipc_Cypipe_Isr_Enable(void)
{
#if (CY_BLE_CONFIG_STACK_CONTR_CORE)
    (void)Cy_IPC_Pipe_Continue(CY_BLE_IPC_CONTROLLER_ADDR, 0u);
#endif  /* (CY_BLE_CONFIG_STACK_CONTR_CORE) */
#if (CY_BLE_HOST_CORE)
    (void)Cy_IPC_Pipe_Continue(CY_BLE_IPC_HOST_ADDR, 0u);
#endif /* (CY_BLE_HOST_CORE) */
}


/*******************************************************************************
* Function Name: Cy_BLE_Ipc_Cypipe_Isr_Disable
****************************************************************************//**
*
*   Disables the IPC pipe Interrupt in the interrupt controller.
*
*******************************************************************************/
void Cy_BLE_Ipc_Cypipe_Isr_Disable(void)
{
#if (CY_BLE_CONFIG_STACK_CONTR_CORE)
    (void)Cy_IPC_Pipe_Pause(CY_BLE_IPC_CONTROLLER_ADDR, 0u);
#endif  /* (CY_BLE_CONFIG_STACK_CONTR_CORE) */
#if (CY_BLE_HOST_CORE)
    (void)Cy_IPC_Pipe_Pause(CY_BLE_IPC_HOST_ADDR, 0u);
#endif /* (CY_BLE_HOST_CORE) */
}

#if (CY_BLE_CONFIG_STACK_CONTR_CORE)


/*******************************************************************************
* Function Name: Cy_BLE_IPC_ControllerRegisterClientCallbacks
****************************************************************************//**
*
* This function registers a callbacks to be called when a message is received on a pipe.
*
* \param ctrlMsgRecvCallBack
* Pointer to the callback to be called when the handle has received a message.
*
* \param ctrlMsgFlushRecvCallBack
* Pointer to the callback to be called when the handle has received a flush message.
*
*  \return
*    CY_IPC_PIPE_SUCCESS       - Callback registered successfully
*    CY_IPC_PIPE_ERROR_BAD_CLIENT - Client ID out of range, callback not registered.
*******************************************************************************/
cy_en_ipc_pipe_status_t Cy_BLE_IPC_ControllerRegisterClientCallbacks(cy_ipc_pipe_callback_ptr_t ctrlMsgRecvCallBack,
                                                                     cy_ipc_pipe_callback_ptr_t ctrlMsgFlushRecvCallBack)
{
    cy_en_ipc_pipe_status_t returnStatus;

    returnStatus = Cy_IPC_Pipe_RegisterCallback(CY_BLE_IPC_CONTROLLER_ADDR, ctrlMsgRecvCallBack,
                                                CY_BLE_CYPIPE_MSG_SEND_ID);

    if(returnStatus == CY_IPC_PIPE_SUCCESS)
    {
        returnStatus = Cy_IPC_Pipe_RegisterCallback(CY_BLE_IPC_CONTROLLER_ADDR, ctrlMsgFlushRecvCallBack,
                                                    CY_BLE_CYPIPE_MSG_COMPLETE_ID);
    }

    return(returnStatus);
}


/*******************************************************************************
* Function Name: Cy_BLE_IPC_SendMessageToHost
****************************************************************************//**
*
* This function sends a message from the Controller to Host.
*
* \param msg
* Pointer to the message structure to be sent.
*
* \param controllerIpcRelCallBack
* Pointer to the Release callback function.
*
* \param controllerPollCallBack
* Pointer to the callback to be called when the handle has received a message.
*
*  \return
*    CY_IPC_PIPE_SUCCESS - message was sent to the other end of the pipe
*    CY_IPC_PIPE_BAD_HANDLE - the handle provided for the pipe was not valid
*    CY_IPC_PIPE_SEND_BUSY - the pipe is already busy sending a message
*    CY_IPC_PIPE_DIR_ERROR - tried to send on the "to" end of a unidirectional pipe
*******************************************************************************/
cy_en_ipc_pipe_status_t Cy_BLE_IPC_SendMessageToHost(uint32_t *msg,
                                                     cy_ipc_pipe_relcallback_ptr_t controllerIpcRelCallBack,
                                                     cy_ipc_pipe_relcallback_ptr_t controllerPollCallBack)
{
    cy_en_ipc_pipe_status_t returnStatus;

    returnStatus = Cy_IPC_Pipe_SendMessage(CY_BLE_IPC_HOST_ADDR, CY_BLE_IPC_CONTROLLER_ADDR, msg,
                                           controllerIpcRelCallBack);

    if(returnStatus != CY_IPC_PIPE_SUCCESS)
    {
        Cy_IPC_Pipe_RegisterCallbackRel(CY_BLE_IPC_CONTROLLER_ADDR, controllerPollCallBack);
    }

    return(returnStatus);
}

#endif /* (CY_BLE_CONFIG_STACK_CONTR_CORE) */

#if (CY_BLE_HOST_CORE)
/*******************************************************************************
* Function Name: Cy_BLE_IPC_HostRegisterClientCallbacks
****************************************************************************//**
*
* This function registers a callbacks to be called when a message is received on a pipe.
*
* \param hostMsgRecvCallBack
* Pointer to the callback to be called when the handle has received a message.
*
* \param hostMsgFlushRecvCallBack
* Pointer to the callback to be called when the handle has received a flush message.
*
*  \return
*    CY_IPC_PIPE_SUCCESS       - Callback registered successfully
*    CY_IPC_PIPE_ERROR_BAD_CLIENT - Client ID out of range, callback not registered.
*******************************************************************************/
cy_en_ipc_pipe_status_t Cy_BLE_IPC_HostRegisterClientCallbacks(cy_ipc_pipe_callback_ptr_t hostMsgRecvCallBack,
                                                               cy_ipc_pipe_callback_ptr_t hostMsgFlushRecvCallBack)
{
    cy_en_ipc_pipe_status_t returnStatus;

    returnStatus = Cy_IPC_Pipe_RegisterCallback(CY_BLE_IPC_HOST_ADDR, hostMsgRecvCallBack, CY_BLE_CYPIPE_MSG_SEND_ID);

    if(returnStatus == CY_IPC_PIPE_SUCCESS)
    {
        returnStatus = Cy_IPC_Pipe_RegisterCallback(CY_BLE_IPC_HOST_ADDR, hostMsgFlushRecvCallBack,
                                                    CY_BLE_CYPIPE_MSG_COMPLETE_ID);
    }

    return(returnStatus);
}


/*******************************************************************************
* Function Name: Cy_BLE_IPC_SendMessageToController
****************************************************************************//**
*
* This function sends a message from the Host to Controller.
*
* \param msg
* Pointer to the message structure to be sent.
*
* \param hostIpcRelCallBack
* Pointer to the Release callback function.
*
* \param hostPollCallBack
* Pointer to the callback to be called when the handle has received a message.
*
*  \return
*    CY_IPC_PIPE_SUCCESS - message was sent to the other end of the pipe
*    CY_IPC_PIPE_BAD_HANDLE - the handle provided for the pipe was not valid
*    CY_IPC_PIPE_SEND_BUSY - the pipe is already busy sending a message
*    CY_IPC_PIPE_DIR_ERROR - tried to send on the "to" end of a unidirectional pipe
*******************************************************************************/
cy_en_ipc_pipe_status_t Cy_BLE_IPC_SendMessageToController(uint32_t *msg,
                                                           cy_ipc_pipe_relcallback_ptr_t hostIpcRelCallBack,
                                                           cy_ipc_pipe_relcallback_ptr_t hostPollCallBack)
{
    cy_en_ipc_pipe_status_t returnStatus;

    returnStatus = Cy_IPC_Pipe_SendMessage(CY_BLE_IPC_CONTROLLER_ADDR, CY_BLE_IPC_HOST_ADDR, msg, hostIpcRelCallBack);

    if(returnStatus != CY_IPC_PIPE_SUCCESS)
    {
        Cy_IPC_Pipe_RegisterCallbackRel(CY_BLE_IPC_HOST_ADDR, hostPollCallBack);
    }

    return(returnStatus);
}
#endif /* (CY_BLE_HOST_CORE) */

#endif /* (CY_BLE_STACK_MODE_IPC) */

/* Mapping functions for stack size optimization */
#if (CY_BLE_MODE_PROFILE)

#if (CY_BLE_SECURE_CONN_FEATURE_ENABLED)

void Cy_BLE_HAL_EccHeapInit(uint8_t * heapMem)
{
    Cy_BLE_HAL_MappingEccHeapInit(heapMem);
}

uint16_t Cy_BLE_HAL_EccGetFeatureHeapReq(void)
{
    return(Cy_BLE_HAL_MappingEccGetFeatureHeapReq());
}

cy_en_ble_api_result_t Cy_BLE_HAL_EccGenerateSecurityKeypair(uint8_t p_publicKey[], 
                                                             uint8_t p_privateKey[], 
                                                             uint8_t random[])
{
    return(Cy_BLE_HAL_MappingEccGenerateSecurityKeypair(p_publicKey, p_privateKey, random));
}

cy_en_ble_api_result_t Cy_BLE_HAL_EccGenerateDHKey(const uint8_t p_publicKey[], 
                                                   const uint8_t p_privateKey[], 
                                                   uint8_t p_secret[], uint8_t ci)
{
    return(Cy_BLE_HAL_MappingEccGenerateDHKey(p_publicKey, p_privateKey, p_secret, ci));
}

cy_en_ble_api_result_t Cy_BLE_HAL_EccValidPublicKey(const uint8_t p_publicKey[])
{
    return(Cy_BLE_HAL_MappingEccValidPublicKey(p_publicKey));
}
    
cy_en_ble_api_result_t Cy_BLE_HAL_PairingLocalPublicKeyHandler(void *param)
{
    return(Cy_BLE_HAL_MappingPairingLocalPublicKeyHandler(param));
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingRemoteKeyHandler(void *param)
{
    return(Cy_BLE_HAL_MappingPairingRemoteKeyHandler(param));
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingDhkeyHandler(void *param)
{
    return(Cy_BLE_HAL_MappingPairingDhkeyHandler(param));
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingDhkeyCheckHandler(void *param)
{
    return(Cy_BLE_HAL_MappingPairingDhkeyCheckHandler(param));
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingKeypressNotificationHandler(void *param)
{
    return(Cy_BLE_HAL_MappingPairingKeypressNotificationHandler(param));
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingRandHandler(void *param)
{
    return(Cy_BLE_HAL_MappingPairingRandHandler(param));
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingConfirmHandler(void *param)
{
    return(Cy_BLE_HAL_MappingPairingConfirmHandler(param));
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingLrConfirmingHandler(void *param)
{
    return(Cy_BLE_HAL_MappingPairingLrConfirmingHandler(param));
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingScTbxDhkeyGenerateComplete(void *param)
{
    Cy_BLE_HAL_MappingTbxDhkeyGenerateComplete(param);

    return(CY_BLE_SUCCESS);
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingScTbxLocalPubkeyGenerateComplete(void *param)
{
    Cy_BLE_HAL_MappingTbxLocalPubkeyGenerateComplete(param);

    return(CY_BLE_SUCCESS);
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingScTbxGenerateLocalP256PublicKey(uint8_t param)
{
    return(Cy_BLE_HAL_MappingTbxGenerateLocalP256PublicKey(param));
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingScTbxGenerateDHkey(void  *param1,
                                                            void  *param2,
                                                            uint8_t param3)
{
    return(Cy_BLE_HAL_MappingTbxGenerateDHkey(param1, param2, param3));
}

void Cy_BLE_HAL_SmpScCmacComplete(uint8_t param)
{
    Cy_BLE_HAL_MappingSmpScCmacComplete(param);
}

cy_en_ble_api_result_t Cy_BLE_HAL_SeSmpScUserPasskeyHandler(void *param1,
                                                            void *param2)
{
    return(Cy_BLE_HAL_MappingSeSmpScUserPasskeyHandler(param1, param2));
}

void Cy_BLE_HAL_EccPointMult(uint8_t param)
{
    Cy_BLE_HAL_MappingEccPointMult(param);
}

#else     /* If feature is not required, return error. */

void Cy_BLE_HAL_EccHeapInit(uint8_t * heapMem CY_UNUSED)
{
 
}

uint16_t Cy_BLE_HAL_EccGetFeatureHeapReq(void)
{
    return(0u);
}

cy_en_ble_api_result_t Cy_BLE_HAL_EccGenerateSecurityKeypair(uint8_t p_publicKey[] CY_UNUSED, 
                                                             uint8_t p_privateKey[] CY_UNUSED, 
                                                             uint8_t random[] CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}

cy_en_ble_api_result_t Cy_BLE_HAL_EccGenerateDHKey(const uint8_t p_publicKey[] CY_UNUSED, 
                                                   const uint8_t p_privateKey[] CY_UNUSED, 
                                                   uint8_t p_secret[] CY_UNUSED, uint8_t ci CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}

cy_en_ble_api_result_t Cy_BLE_HAL_EccValidPublicKey(const uint8_t p_publicKey[] CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}    
    
    
cy_en_ble_api_result_t Cy_BLE_HAL_PairingLocalPublicKeyHandler(void *param CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingRemoteKeyHandler(void *param CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingDhkeyHandler(void *param CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingDhkeyCheckHandler(void *param CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingKeypressNotificationHandler(void *param CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingRandHandler(void *param CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingConfirmHandler(void *param CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingLrConfirmingHandler(void *param CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingScTbxDhkeyGenerateComplete(void *param CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingScTbxLocalPubkeyGenerateComplete(void *param CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingScTbxGenerateLocalP256PublicKey(uint8_t param CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}

cy_en_ble_api_result_t Cy_BLE_HAL_PairingScTbxGenerateDHkey(void  *param1 CY_UNUSED,
                                                            void  *param2 CY_UNUSED,
                                                            uint8_t param3 CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}

void Cy_BLE_HAL_SmpScCmacComplete(uint8_t param CY_UNUSED)
{
}

cy_en_ble_api_result_t Cy_BLE_HAL_SeSmpScUserPasskeyHandler(void  *param1 CY_UNUSED,
                                                            void  *param2 CY_UNUSED)
{
    return(CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
}

void Cy_BLE_HAL_EccPointMult(uint8_t param CY_UNUSED)
{
}

#endif /* (CY_BLE_SECURE_CONN_FEATURE_ENABLED) */

#endif /* CY_BLE_MODE_PROFILE */

#endif /* CY_BLE_HOST_CONTR_CORE */

#ifdef __cplusplus
}
#endif /* __cplusplus */

/* [] END OF FILE */
