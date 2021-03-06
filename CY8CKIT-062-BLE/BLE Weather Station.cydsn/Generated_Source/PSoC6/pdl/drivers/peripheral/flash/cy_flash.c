/***************************************************************************//**
* \file cy_flash.c
* \version 1.0
*
* \brief
* Provides the public functions for the API for the PSoC 6 Flash Driver.
*
********************************************************************************
* \copyright
* Copyright 2016-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_flash.h"


/* Static functions */
static uint32_t Cy_Flash_BoundsCheck(uint32_t flashAddr);
static uint32_t Cy_Flash_PrepareContext(uint32_t rowAddr, const uint32_t *data);
static cy_en_flashdrv_status_t Cy_Flash_ProcessOpcode(uint32_t opcode);
static cy_stc_flash_context_t context;

/*******************************************************************************
* Function Name: Cy_Flash_EraseRow
****************************************************************************//**
*
* This function erases a single row of flash. Reports success or
* a reason for failure. Does not return until the Write operation is
* complete. Returns immediately and reports a CY_FLASH_DRV_IPC_BUSY error in
* the case when another process is writing to flash or erasing the row.
* User firmware should not enter the hibernate mode until flash erase is
* complete. The Flash operation is allowed in Sleep and Deep-sleep modes.
* During the Flash operation, the device should not be reset, including the
* XRES pin, a software reset, and watchdog reset sources. Also, low-voltage
* detect circuits should be configured to generate an interrupt instead of a
* reset. Otherwise, portions of flash may undergo unexpected changes.
*
* \param rowAddr Address of the flash row number. The number of the flash rows
* is defined by the \ref CY_FLASH_NUMBER_ROWS macro for the selected device.
* The Read-while-Write violation occurs when the flash read operation is
* initiated in the same flash sector where the flash write operation is
* performing. Refer to the device datasheet for the details.
* Address must match row start address.
*
* \return Returns the status of the Flash operation (see \ref cy_en_flashdrv_status_t).
*
*******************************************************************************/
cy_en_flashdrv_status_t Cy_Flash_EraseRow(uint32_t rowAddr)
{
    uint32_t result;
    cy_en_flashdrv_status_t retVal;

    /* Prepares arguments to be passed to SROM API */
    if (Cy_Flash_BoundsCheck(rowAddr) != 0UL)
    {
        context.opcode = CY_FLASH_OPCODE_ERASE_ROW;
        context.arg1 = rowAddr;
        context.arg2 = 0UL;
        context.arg3 = 0UL;
#if (CY_CPU_CORTEX_M0P)
        uint32_t interruptState;
        uint32_t regVal;

        interruptState = Cy_SysLib_EnterCriticalSection();

        regVal = Cy_IPC_Drv_GetIntrMask(IPC_INTR_STRUCT0);
        Cy_IPC_Drv_SetIntrMask(IPC_INTR_STRUCT0, ((regVal >> IPC_INTR_STRUCT_INTR_NOTIFY_Pos) | CY_FLASH_IPC_INT_MASK), 
                                                    ((regVal & IPC_INTR_STRUCT_INTR_MASK_RELEASE_Msk) | CY_FLASH_IPC_INT_MASK));
        
        /* Tries to acquire the IPC structure and pass the arguments to SROM API */
        if (Cy_IPC_Drv_SendMsgPtr(CY_FLASH_IPC_STRUCT, CY_FLASH_IPC_NOTIFY_STRUCT0, (void*)&context) == CY_IPC_DRV_SUCCESS)
        {
            /* The Flash operation is successfully initiated */
            result = (uint32_t)CY_FLASH_DRV_OPERATION_STARTED;

            /* Polls whether IPC is released and the Flash operation is performed */
            do
            {
                result = (uint32_t)Cy_Flash_IsWriteComplete();
            }
            while (result == (uint32_t)CY_FLASH_DRV_OPCODE_BUSY);
        }
        else
        {
            /* The IPC structure is already locked by another process */
            result = (uint32_t)CY_FLASH_DRV_IPC_BUSY;
        }
        
        Cy_SysLib_ExitCriticalSection(interruptState);
#else
        Cy_Flash_Proxy Cy_Flash_Cm4Proxy;
        Cy_Flash_Cm4Proxy = (Cy_Flash_Proxy)CY_FLASH_CM4_FLASH_PROXY_ADDR;
        result = (uint32_t)Cy_Flash_Cm4Proxy(&context);

#endif /* (CY_CPU_CORTEX_M0P) */
    }
    else
    {
        result = (uint32_t)CY_FLASH_DRV_INVALID_INPUT_PARAMETERS;
    }

    /* Invalidates the flash cache and buffer */
    Cy_SysLib_ClearFlashCacheAndBuffer() ;

    retVal = Cy_Flash_ProcessOpcode(result);

    return (retVal);
}


/*******************************************************************************
* Function Name: Cy_Flash_WriteRow
****************************************************************************//**
*
* This function writes an array of data to a single row of flash. Reports success
* or a reason for failure. Does not return until the Write operation is complete.
* Returns immediately and reports a CY_FLASH_DRV_IPC_BUSY error in the case when another
* process is writing to flash. User firmware should not enter the hibernate
* mode until flash Write is complete. The Flash operation is allowed in Sleep and
* Deep-sleep modes. During the Flash operation, the device should not be reset,
* including the XRES pin, a software reset, and watchdog reset sources. Also,
* low-voltage detect circuits should be configured to generate an interrupt instead
* of a reset. Otherwise, portions of flash may undergo unexpected changes.
*
* \param rowAddr Address of the flash row number. The number of the flash rows
* is defined by the \ref CY_FLASH_NUMBER_ROWS macro for the selected device.
* The Read-while-Write violation occurs when the flash read operation is
* initiated in the same flash sector where the flash write operation is
* performing. Refer to the device datasheet for the details.
* Address must match row start address.
*
* \param data The pointer to the data which has to be written to flash. The size
* of the data array must be equal to the flash row size. The flash row size for
* the selected device is defined by the \ref CY_FLASH_SIZEOF_ROW macro. Refer to
* the device datasheet for the details.
*
* \return Returns the status of the Flash operation (see \ref cy_en_flashdrv_status_t).
*
*******************************************************************************/
cy_en_flashdrv_status_t Cy_Flash_WriteRow(uint32_t rowAddr, const uint32_t* data)
{
    uint32_t result;
    cy_en_flashdrv_status_t retVal;
    
#if (CY_CPU_CORTEX_M0P)
    uint32_t interruptState;
    uint32_t regVal;

    interruptState = Cy_SysLib_EnterCriticalSection();

    regVal = Cy_IPC_Drv_GetIntrMask(IPC_INTR_STRUCT0);
    Cy_IPC_Drv_SetIntrMask(IPC_INTR_STRUCT0, ((regVal >> IPC_INTR_STRUCT_INTR_NOTIFY_Pos) | 0x07u), \
                                                ((regVal & IPC_INTR_STRUCT_INTR_MASK_RELEASE_Msk) | 0x07u));
    
    result = (uint32_t)Cy_Flash_StartWrite(rowAddr, data);
    
    /* Check whether the Flash operation was successfully initiated */
    if (result == (uint32_t)CY_FLASH_DRV_OPERATION_STARTED)
    {
        /* Polls whether IPC is released and the Flash operation is performed */
        do
        {
            result = (uint32_t)Cy_Flash_IsWriteComplete();
        }
        while (result == (uint32_t)CY_FLASH_DRV_OPCODE_BUSY);
    }

    Cy_SysLib_ExitCriticalSection(interruptState);
#else
    if (Cy_Flash_PrepareContext(rowAddr, data) != (uint32_t)CY_FLASH_DRV_INVALID_INPUT_PARAMETERS)
    {
        Cy_Flash_Proxy Cy_Flash_Cm4Proxy;
        Cy_Flash_Cm4Proxy = (Cy_Flash_Proxy)CY_FLASH_CM4_FLASH_PROXY_ADDR;
        result = (uint32_t)Cy_Flash_Cm4Proxy(&context);
    }
    else
    {
        result = (uint32_t)CY_FLASH_DRV_INVALID_INPUT_PARAMETERS;
    }
#endif /* (CY_CPU_CORTEX_M0P) */

    /* Invalidates the flash cache and buffer */
    FLASHC->FLASH_CMD = FLASHC_FLASH_CMD_INV_Msk;

    retVal = Cy_Flash_ProcessOpcode(result);

    return (retVal);
}


/*******************************************************************************
* Function Name: Cy_Flash_StartWrite
****************************************************************************//**
*
* Starts writing an array of data to a single row of flash. Returns immediately
* and reports a successful start or reason for failure. Returns immediately and
* reports a CY_FLASH_IPC_BUSY error in the case when another process is writing
* to flash. User firmware should not enter the hibernate mode until flash Write
* is complete. The Flash operation is allowed in Sleep and Deep-sleep modes.
* During the flash operation, the device should not be reset, including the
* XRES pin, a software reset, and watchdog reset sources. Also, the low-voltage
* detect circuits should be configured to generate an interrupt instead of a reset.
* Otherwise, portions of flash may undergo unexpected changes.
*
* \param rowAddr Address of the flash row number. The number of the flash rows
* is defined by the \ref CY_FLASH_NUMBER_ROWS macro for the selected device.
* The Read-while-Write violation occurs when the flash read operation is
* initiated in the same flash sector where the flash write operation is
* performing. Refer to the device datasheet for the details.
* Address must row start address.
*
* \param data The pointer to the data to be written to flash. The size
* of the data array must be equal to the flash row size. The flash row size for
* the selected device is defined by the \ref CY_FLASH_SIZEOF_ROW macro. Refer to
* the device datasheet for the details.
*
* \return Returns the status of the Flash operation (see \ref cy_en_flashdrv_status_t).
*
*******************************************************************************/
cy_en_flashdrv_status_t Cy_Flash_StartWrite(uint32_t rowAddr, const uint32_t* data)
{
    cy_en_flashdrv_status_t result;

    /* Checks whether the input parameters are valid */
    if (Cy_Flash_PrepareContext(rowAddr, data) != (uint32_t)CY_FLASH_DRV_INVALID_INPUT_PARAMETERS)
    {
        /* Tries to acquire the IPC structure and pass the arguments to SROM API */
        if (Cy_IPC_Drv_SendMsgPtr(CY_FLASH_IPC_STRUCT, CY_FLASH_IPC_NOTIFY_STRUCT0, (void*)&context) == CY_IPC_DRV_SUCCESS)
        {
            /* The Flash operation is successfully initiated */
            result = CY_FLASH_DRV_OPERATION_STARTED;
        }
        else
        {
            /* The IPC structure is already locked by another process */
            result = CY_FLASH_DRV_IPC_BUSY;
        }
    }
    else
    {
        result = CY_FLASH_DRV_INVALID_INPUT_PARAMETERS;
    }

    return (result);
}


/*******************************************************************************
* Function Name: Cy_Flash_IsWriteComplete
****************************************************************************//**
*
* Reports a successful Write, reason of failure or busy status (CY_FLASH_DRV_OPCODE_BUSY).
*
* \return Returns the status of the Flash operation (see \ref cy_en_flashdrv_status_t).
*
*******************************************************************************/
cy_en_flashdrv_status_t Cy_Flash_IsWriteComplete(void)
{
    cy_en_flashdrv_status_t retVal = CY_FLASH_DRV_OPCODE_BUSY;

    /* Checks if the IPC structure is not locked */
    if (Cy_IPC_Drv_IsLockAcquired(CY_FLASH_IPC_STRUCT) == false)
    {
        /* The result of SROM API calling is returned to the driver context */
        retVal = Cy_Flash_ProcessOpcode(context.opcode);
    }

    return (retVal);
}


/*******************************************************************************
* Function Name: Cy_Flash_RowChecksum
****************************************************************************//**
*
* Returns a checksum value of the specified flash row.
*
* \param rowNum The Cheksum is calculated to the flash row.
*
* \param cheksumPtr The pointer to the address where cheksum is to be stored
*
* \return Returns the status of the Flash operation.
*
*******************************************************************************/
cy_en_flashdrv_status_t Cy_Flash_RowChecksum (uint32_t rowNum, uint32_t* cheksumPtr)
{
    cy_en_flashdrv_status_t result = CY_FLASH_DRV_INVALID_INPUT_PARAMETERS;
    uint32_t resTmp;

    /* Checks whether the input parameters are valid */
    if (rowNum >= CY_FLASH_NUMBER_ROWS)
    {
        result = CY_FLASH_DRV_INVALID_INPUT_PARAMETERS;
    }
    else
    {
        /* Prepares arguments to be passed to SROM API */
        context.opcode = CY_FLASH_OPCODE_CHECKSUM | (rowNum << CY_FLASH_OPCODE_CHECKSUM_ROW_SHIFT);

        /* Tries to acquire the IPC structure and pass the arguments to SROM API */
        if (Cy_IPC_Drv_SendMsgPtr(CY_FLASH_IPC_STRUCT, CY_FLASH_IPC_NOTIFY_STRUCT0, (void*)&context) == CY_IPC_DRV_SUCCESS)
        {
            /* Polls whether IPC is released and the Flash operation is performed */
            while (Cy_IPC_Drv_IsLockAcquired(CY_FLASH_IPC_STRUCT) != false)
            {
                /* Wait till IPC is released */
            }

            resTmp = context.opcode;

            if((resTmp >> CY_FLASH_ERROR_SHIFT) == CY_FLASH_ERROR_NO_ERROR)
            {
                result = CY_FLASH_DRV_SUCCESS;
                *cheksumPtr = context.opcode & CY_FLASH_RESULT_MASK;
            }
            else
            {
                result = Cy_Flash_ProcessOpcode((uint32_t)result);
            }
        }
        else
        {
            /* The IPC structure is already locked by another process */
            result = CY_FLASH_DRV_IPC_BUSY;
        }
    }

    return (result);
}


/*******************************************************************************
* Function Name: Cy_Flash_CalculateHash
****************************************************************************//**
*
* Returns a hash value of the specified region of flash.
*
* \param data Start the data address.
*
* \param numberOfBytes The hash value is calculated for the number of bytes after the
* start data address (0 - 1 byte, 1- 2 bytes etc).
*
* \param hashPtr The pointer to the address where hash is to be stored
*
* \return Returns the status of the Flash operation.
*
*******************************************************************************/
cy_en_flashdrv_status_t Cy_Flash_CalculateHash (const uint32_t* data, uint32 numberOfBytes,  uint32_t* hashPtr)
{
    cy_en_flashdrv_status_t result = CY_FLASH_DRV_INVALID_INPUT_PARAMETERS;
    volatile uint32_t resTmp;

    /* Checks whether the input parameters are valid */
    if (data == NULL)
    {
        result = CY_FLASH_DRV_INVALID_INPUT_PARAMETERS;
    }
    else
    {
        /* Prepares arguments to be passed to SROM API */
        context.opcode = CY_FLASH_OPCODE_HASH;
        context.arg1 = (uint32_t)data;
        context.arg2 = numberOfBytes;

        /* Tries to acquire the IPC structure and pass the arguments to SROM API */
        if (Cy_IPC_Drv_SendMsgPtr(CY_FLASH_IPC_STRUCT, CY_FLASH_IPC_NOTIFY_STRUCT0, (void*)&context) == CY_IPC_DRV_SUCCESS)
        {
            /* Polls whether IPC is released and the Flash operation is performed */
            while (Cy_IPC_Drv_IsLockAcquired(CY_FLASH_IPC_STRUCT) != false)
            {
                /* Wait till IPC is released */
            }

            resTmp = context.opcode;

            if((resTmp >> CY_FLASH_ERROR_SHIFT) == CY_FLASH_ERROR_NO_ERROR)
            {
                result = CY_FLASH_DRV_SUCCESS;
                *hashPtr = context.opcode & CY_FLASH_RESULT_MASK;
            }
            else
            {
                result = Cy_Flash_ProcessOpcode((uint32_t)result);
            }
        }
        else
        {
            /* The IPC structure is already locked by another process */
            result = CY_FLASH_DRV_IPC_BUSY;
        }
    }

    return (result);
}


/*******************************************************************************
* Function Name: Cy_Flash_BoundsCheck
****************************************************************************//**
*
* Returns 1 if Flash addres is out of boundary, otherwise returns 0.
*
* \param flashAddr Address to be checked
*
* \return 1 - out of bound, 0 - in flash bounds
*
*******************************************************************************/
static uint32_t Cy_Flash_BoundsCheck(uint32_t flashAddr)
{
    uint32_t result = 1UL;

    if ((flashAddr < CY_FLASH_BASE) || (flashAddr >= (CY_FLASH_BASE + CY_FLASH_SIZE)))
    {
        if ((flashAddr < CY_WFLASH_BASE) || (flashAddr >= (CY_WFLASH_BASE + CY_WFLASH_SIZE)))
        {
            if ((flashAddr < SFLASH_BASE) || (flashAddr >= (SFLASH_BASE + SFLASH_SECTION_SIZE)))
            {
                result = 0UL;
            }
        }
    }

    return (result);
}


/*******************************************************************************
* Function Name: Cy_Flash_PrepareContext
****************************************************************************//**
*
* Returns 1 if parameters are correct and initialises context, otherwise returns
* CY_FLASH_DRV_INVALID_INPUT_PARAMETERS.
*
* \param rowAddr Address of the flash row number. The number of the flash rows
* is defined by the \ref CY_FLASH_NUMBER_ROWS macro for the selected device.
* The Read-while-Write violation occurs when the flash read operation is
* initiated in the same flash sector where the flash write operation is
* performing. Refer to the device datasheet for the details.
* Address must match row start address.
*
* \param data The pointer to the data which has to be written to flash. The size
* of the data array must be equal to the flash row size. The flash row size for
* the selected device is defined by the \ref CY_FLASH_SIZEOF_ROW macro. Refer to
* the device datasheet for the details.
*
* \return 1 - Context ready, CY_FLASH_DRV_INVALID_INPUT_PARAMETERS - Wrong arguments
*
*******************************************************************************/
static uint32_t Cy_Flash_PrepareContext(uint32_t rowAddr, const uint32_t* data)
{
    uint32_t result;

    if ((Cy_Flash_BoundsCheck(rowAddr) == 0UL) || (NULL == data))
    {
        result = (uint32_t)CY_FLASH_DRV_INVALID_INPUT_PARAMETERS;
    }
    else
    {
        if ((rowAddr%CY_FLASH_SIZEOF_ROW) != 0UL)
        {
            result = (uint32_t)CY_FLASH_DRV_INVALID_INPUT_PARAMETERS;
        }
        else
        {
            /* Prepares arguments to be passed to SROM API */
            context.opcode = CY_FLASH_OPCODE_WRITE_ROW;
            context.arg1 = CY_FLASH_CONFIG_DATASIZE;
            context.arg2 = rowAddr;
            context.arg3 = (uint32_t)data;

            result = 1UL;
        }
    }

    return (result);
}


/*******************************************************************************
* Function Name: Cy_Flash_ProcessOpcode
****************************************************************************//**
*
* Converts System Call returns to the Flash driver return defines.
*
* \param opcode The value returned by the System Call.
*
* \return Flash driver return.
*
*******************************************************************************/
static cy_en_flashdrv_status_t Cy_Flash_ProcessOpcode(uint32_t opcode)
{
    cy_en_flashdrv_status_t retVal;

    switch (opcode)
    {
        case 0UL:
        {
            retVal = CY_FLASH_DRV_SUCCESS;
            break;
        }
        case CY_FLASH_ROMCODE_SUCCESS:
        {
            retVal = CY_FLASH_DRV_SUCCESS;
            break;
        }
        case CY_FLASH_ROMCODE_INVALID_PROTECTION:
        {
            retVal = CY_FLASH_DRV_INV_PROT;
            break;
        }
        case CY_FLASH_ROMCODE_INVALID_FM_PL:
        {
            retVal = CY_FLASH_DRV_INVALID_FM_PL;
            break;
        }
        case CY_FLASH_ROMCODE_INVALID_FLASH_ADDR:
        {
            retVal = CY_FLASH_DRV_INVALID_FLASH_ADDR;
            break;
        }
        case CY_FLASH_ROMCODE_ROW_PROTECTED:
        {
            retVal = CY_FLASH_DRV_ROW_PROTECTED;
            break;
        }
        case CY_FLASH_ROMCODE_IN_PROGRESS_NO_ERROR:
        {
            retVal = CY_FLASH_DRV_PROGRESS_NO_ERROR;
            break;
        }
        case (uint32_t)CY_FLASH_DRV_INVALID_INPUT_PARAMETERS:
        {
            retVal = CY_FLASH_DRV_INVALID_INPUT_PARAMETERS;
            break;
        }
        default:
        {
            retVal = CY_FLASH_DRV_ERR_UNC;
            break;
        }
    }

    return (retVal);
}


/* [] END OF FILE */
