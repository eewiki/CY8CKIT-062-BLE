/***************************************************************************//**
* \file cy_ipc_pipe.c
* \version 1.0
*
*  Description:
*   IPC Pipe Driver - This source file includes code for the Pipe layer on top
*   of the IPC driver.
*
********************************************************************************
* Copyright 2016-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_ipc_pipe.h"

/* Define a pointer to array of endPoints. */
static cy_stc_ipc_pipe_ep_t * cy_ipc_pipe_epArray = NULL;

/*******************************************************************************
* Function Name: Cy_IPC_Pipe_Config
****************************************************************************//**
*
* This function stores a copy of a pointer to the array of endpoints.  All
* access to endpoints will be via the index of the endpoint in this array.
*
* \param theEpArray
* This is the pointer to an array of endpoint structures that the designer
* created and will be used to reference all endpoints.
*
*******************************************************************************/
void Cy_IPC_Pipe_Config(cy_stc_ipc_pipe_ep_t * theEpArray)
{
    /* Keep copy of this endpoint */
    if (cy_ipc_pipe_epArray == NULL)
    {
        cy_ipc_pipe_epArray = theEpArray;
    }
}

/*******************************************************************************
* Function Name: Cy_IPC_SystemPipeInit
****************************************************************************//**
*
* Initializes the system pipes. The system pipes are used by BLE.
* \note The function should be called on all CPUs.
*
* \param config
* This is the pointer to the pipe configuration structure
*
*******************************************************************************/
void Cy_IPC_Pipe_Init(cy_stc_ipc_pipe_config_t const *config)
{
    /* Create the interrupt structures and arrays needed */

    cy_stc_sysint_t                 ipc_intr_cypipeConfig;

    cy_stc_ipc_pipe_ep_config_t        epConfigDataA;
    cy_stc_ipc_pipe_ep_config_t        epConfigDataB;

#if (CY_CPU_CORTEX_M0P)

    /* Receiver endpoint = EP0, Sender endpoint = EP1 */
    epConfigDataA = config->ep0ConfigData;
    epConfigDataB = config->ep1ConfigData;

    /* Configure CM0 interrupts */
    ipc_intr_cypipeConfig.intrSrc          = (IRQn_Type)epConfigDataA.ipcNotifierMuxNumber;
    ipc_intr_cypipeConfig.cm0pSrc          = (cy_en_intr_t)((int32_t)cpuss_interrupts_ipc0_IRQn + epConfigDataA.ipcNotifierNumber);
    ipc_intr_cypipeConfig.intrPriority     = epConfigDataA.ipcNotifierPriority;

#else

    /* Receiver endpoint = EP1, Sender endpoint = EP0 */
    epConfigDataA = config->ep1ConfigData;
    epConfigDataB = config->ep0ConfigData;

    /* Configure interrupts */
    ipc_intr_cypipeConfig.intrSrc          = (IRQn_Type)(cpuss_interrupts_ipc0_IRQn + epConfigDataA.ipcNotifierNumber);
    ipc_intr_cypipeConfig.intrPriority     = epConfigDataA.ipcNotifierPriority;

#endif

    /* Initialize the pipe endpoints */
    Cy_IPC_Pipe_EndpointInit(epConfigDataA.epAddress,
                             config->endpointsCallbacksArray,
                             config->endpointClientsCount,
                             epConfigDataA.epConfig,
                             &ipc_intr_cypipeConfig);

    /* Create the endpoints for the CM4 just for reference */
    Cy_IPC_Pipe_EndpointInit(epConfigDataB.epAddress, (void *)0, 0ul, epConfigDataB.epConfig, NULL);

    (void)Cy_SysInt_Init(&ipc_intr_cypipeConfig, config->userPipeIsrHandler);

    /* Enable the interrupts */
    NVIC_EnableIRQ(ipc_intr_cypipeConfig.intrSrc);
}

/*******************************************************************************
* Function Name: Cy_IPC_Pipe_EndpointInit
****************************************************************************//**
*
* This function initializes the endpoint of a pipe for the current CPU.  The 
* current CPU is the CPU that is executing the code. An endpoint of a pipe
* is for the IPC channel that receives a message for the current CPU.
*
* After this function is called, the callbackArray needs to be populated
* with the callback functions for that endpoint using the
* Cy_IPC_Pipe_RegisterCallback() function.
*
* \param epAddr
* This parameter is the address (or index in the array of endpoint structures)
* that designates the endpoint you want to initialize.
*
* \param cbArray
* This is a pointer to the callback function array.  Based on the client ID, one
* of the functions in this array is called to process the message.
*
* \param cbCnt
* This is the size of the callback array, or the number of defined clients.
*
* \param epConfig
* This value defines the IPC channel, IPC interrupt number, and the interrupt
* mask for the entire pipe.
* The format of the endpoint configuration
*    Bits[31:16] Interrupt Mask
*    Bits[15:8 ] IPC interrupt
*    Bits[ 7:0 ] IPC channel
*
* \param epInterrupt
* This is a pointer to the endpoint interrupt description structure.
*
*  \return
*   void
*
*******************************************************************************/
void Cy_IPC_Pipe_EndpointInit(uint32_t epAddr, cy_ipc_pipe_callback_array_ptr_t cbArray,
                              uint32_t cbCnt, uint32_t epConfig, cy_stc_sysint_t const *epInterrupt)
{
    cy_stc_ipc_pipe_ep_t * endpoint;

    endpoint = &cy_ipc_pipe_epArray[epAddr];

    /* Extract the channel, interrupt and interrupt mask */
    endpoint->ipcChan         = _FLD2VAL(CY_IPC_PIPE_CFG_CHAN,  epConfig);
    endpoint->intrChan        = _FLD2VAL(CY_IPC_PIPE_CFG_INTR,  epConfig);
    endpoint->pipeIntMask     = _FLD2VAL(CY_IPC_PIPE_CFG_IMASK, epConfig);

    /* Assign IPC channel to this endpoint */
    endpoint->ipcPtr   = Cy_IPC_Drv_GetIpcBaseAddress (endpoint->ipcChan);

    /* Assign interrupt structure to endpoint and Initialize the interrupt mask for this endpoint */
    endpoint->ipcIntrPtr = Cy_IPC_Drv_GetIntrBaseAddr(endpoint->intrChan);

    /* Only allow notify and release interrupts from endpoints in this pipe. */
    Cy_IPC_Drv_SetInterruptMask(endpoint->ipcIntrPtr, endpoint->pipeIntMask, endpoint->pipeIntMask);

    /* Save the Client count and the callback array pointer */
    endpoint->clientCount   = cbCnt;
    endpoint->callbackArray = cbArray;
    endpoint->busy = CY_IPC_PIPE_ENDPOINT_NOTBUSY;

    if (NULL != epInterrupt)
    {
        endpoint->pipeIntrSrc     = epInterrupt->intrSrc;
    }
}


/*******************************************************************************
* Function Name: Cy_IPC_Pipe_SendMessage
****************************************************************************//**
*
* This function is used to send a message from one endpoint to another.  It
* generates an interrupt on the endpoint that receives the message and a
* release interrupt to the sender to acknowledge the message has been processed.
*
* \param toAddr
* This parameter is the address (or index in the array of endpoint structures)
* of the endpoint to which you are sending the message.
*
* \param fromAddr
* This parameter is the address (or index in the array of endpoint structures)
* of the endpoint from which the message is being sent.
*
* \param msgPtr
* Pointer to the message structure to be sent.
*
* \param callBackPtr
* Pointer to the Release callback function.
*
*
*  \return
*    CY_IPC_PIPE_SUCCESS:     Message was sent to the other end of the pipe
*    CY_IPC_PIPE_BAD_HANDLE:  The handle provided for the pipe was not valid
*    CY_IPC_PIPE_SEND_BUSY:   The pipe is already busy sending a message
*    CY_IPC_PIPE_DIR_ERROR:   Tried to send on the "to" end of a unidirectional pipe
*******************************************************************************/
cy_en_ipc_pipe_status_t Cy_IPC_Pipe_SendMessage(uint32_t toAddr, uint32_t fromAddr, void * msgPtr, cy_ipc_pipe_relcallback_ptr_t callBackPtr)
{
    cy_en_ipc_pipe_status_t  returnStatus;
    uint32_t releaseMask;
    uint32_t notifyMask;

    cy_stc_ipc_pipe_ep_t * fromEp;
    cy_stc_ipc_pipe_ep_t * toEp;

    toEp   = &(cy_ipc_pipe_epArray[toAddr]);
    fromEp = &cy_ipc_pipe_epArray[fromAddr];

    /* Create the release mask for the "fromAddr" channel's interrupt channel */
    releaseMask =  (uint32_t)(1ul << (fromEp->intrChan));

    /* Shift into position */
    releaseMask = _VAL2FLD(CY_IPC_PIPE_MSG_RELEASE, releaseMask);

    /* Create the notify mask for the "toAddr" channel's interrupt channel */
    notifyMask  =  (uint32_t)(1ul << (toEp->intrChan));

    /* Check if IPC channel valid */
    if( toEp->ipcPtr != (void *)0u)
    {
        if(fromEp->busy == CY_IPC_PIPE_ENDPOINT_NOTBUSY)
        {
            /* Attempt to acquire the channel */
            if( CY_IPC_DRV_SUCCESS == Cy_IPC_Drv_LockAcquire(toEp->ipcPtr) )
            {
                /* Mask out the release mask area */
                * (uint32_t *) msgPtr &= ~(CY_IPC_PIPE_MSG_RELEASE_Msk);

                * (uint32_t *) msgPtr |= releaseMask;

                /* If the channel was acquired, write the message.   */
                Cy_IPC_Drv_WriteDataValue(toEp->ipcPtr, (uint32_t) msgPtr);

                /* Set the busy flag.  The ISR clears this after the release */
                fromEp->busy = CY_IPC_PIPE_ENDPOINT_BUSY;

                /* Setup release callback function */
                fromEp->releaseCallbackPtr = callBackPtr;

                /* Cause notify event/interrupt */
                Cy_IPC_Drv_AcquireNotify(toEp->ipcPtr, notifyMask);

                returnStatus = CY_IPC_PIPE_SUCCESS;
            }
            else
            {
                /* Channel was already acquired, return Error */
                returnStatus = CY_IPC_PIPE_ERROR_SEND_BUSY;
            }
        }
        else
        {
            /* Channel may not be acquired, but the release interrupt has not executed yet */
            returnStatus = CY_IPC_PIPE_ERROR_SEND_BUSY;
        }
    }
    else
    {
        /* Null pipe handle. */
        returnStatus = CY_IPC_PIPE_ERROR_BAD_HANDLE;
    }
    return (returnStatus);
}



/*******************************************************************************
* Function Name: Cy_IPC_Pipe_RegisterCallback
****************************************************************************//**
*
* This function registers a callback that is called when a message is received on a pipe.
* The client_ID is the same as the index of the callback function array.  The callback
* may be a real function pointer or NULL if no callback is required.
*
* \param epAddr
* This parameter is the address (or index in the array of endpoint structures)
* that designates the endpoint to which you want to add callback functions.
*
* \param callBackPtr
* Pointer to the callback function called when the endpoint has received a message.
*
* \param clientId
* The index in the callback array (Client ID) where the function pointer is saved.
*
*  \return
*    CY_IPC_PIPE_SUCCESS:           Callback registered successfully
*    CY_IPC_PIPE_ERROR_BAD_CLIENT:  Client ID out of range, callback not registered.
*******************************************************************************/
cy_en_ipc_pipe_status_t Cy_IPC_Pipe_RegisterCallback(uint32_t epAddr, cy_ipc_pipe_callback_ptr_t callBackPtr,  uint32_t clientId)
{
    cy_en_ipc_pipe_status_t returnStatus;
    cy_stc_ipc_pipe_ep_t * thisEp;

    thisEp = &cy_ipc_pipe_epArray[epAddr];

    /* Check if clientId is between 0 and less than client count */
    if (clientId < thisEp->clientCount)
    {
        /* Copy callback function into callback function pointer array */
        thisEp->callbackArray[clientId] = callBackPtr;

        returnStatus = CY_IPC_PIPE_SUCCESS;
    }
    else
    {
        returnStatus = CY_IPC_PIPE_ERROR_BAD_CLIENT;
    }
    return (returnStatus);
}

/*******************************************************************************
* Function Name: Cy_IPC_Pipe_RegisterCallbackRel
****************************************************************************//**
*
* This function registers a default callback if a release interrupt
* is generated but the current release callback function is null.
*
*
* \param epAddr
* This parameter is the address (or index in the array of endpoint structures)
* that designates the endpoint to which you want to add a release callback function.
*
* \param callBackPtr
* Pointer to the callback executed when the endpoint has received a message.
*
*  \return
*    None
*******************************************************************************/
void Cy_IPC_Pipe_RegisterCallbackRel(uint32_t epAddr, cy_ipc_pipe_relcallback_ptr_t callBackPtr)
{
    cy_stc_ipc_pipe_ep_t * endpoint;

    endpoint = &cy_ipc_pipe_epArray[epAddr];

    /* Copy callback function into callback function pointer array */
    endpoint->defaultReleaseCallbackPtr = callBackPtr;
}

/*******************************************************************************
* Function Name: Cy_IPC_Pipe_ExecCallback
****************************************************************************//**
*
* This function is called by the ISR for a given pipe endpoint to dispatch
* the appropriate callback function based on the client ID for that endpoint.
*
* \param endpoint
* Pointer to endpoint structure.
*
*  \return
*  None
*******************************************************************************/
void Cy_IPC_Pipe_ExecCallback(cy_stc_ipc_pipe_ep_t * endpoint)
{
    uint32_t *msgPtr = (void *)0uL;
    uint32_t clientID;
    uint32_t shadowIntr;
    uint32_t releaseMask = (uint32_t)0;

    cy_ipc_pipe_callback_ptr_t callbackPtr;

    /* Check to make sure the interrupt was a notify interrupt */
    shadowIntr = Cy_IPC_Drv_GetInterruptStatusMasked(endpoint->ipcIntrPtr);

    if( 0ul != (shadowIntr & IPC_INTR_STRUCT_INTR_MASK_NOTIFY_Msk))
    {
        /* Clear the notify interrupt.  */
        endpoint->ipcIntrPtr->INTR = (shadowIntr & IPC_INTR_STRUCT_INTR_MASK_NOTIFY_Msk);

        if( Cy_IPC_Drv_IsLockAcquired (endpoint->ipcPtr) )
        {
            /* Extract Client ID  */
            if( CY_IPC_DRV_SUCCESS == Cy_IPC_Drv_ReadMsgPtr (endpoint->ipcPtr, (void **)&msgPtr))
            {
                /* Get release mask */
                releaseMask = _FLD2VAL(CY_IPC_PIPE_MSG_RELEASE, *msgPtr);
                clientID    = _FLD2VAL(CY_IPC_PIPE_MSG_CLIENT,  *msgPtr);

                /* Make sure client ID is within valid range */
                if (endpoint->clientCount > clientID)
                {
                    callbackPtr = endpoint->callbackArray[clientID];  /* Get the callback function */

                    if (callbackPtr != NULL)
                    {
                        callbackPtr(msgPtr);   /* Call the function pointer for "clientID" */
                    }
                }
            }

            /* Must always release the IPC channel */
            (void)Cy_IPC_Drv_LockRelease (endpoint->ipcPtr, releaseMask);
        }
    }

    /* Check to make sure the interrupt was a release interrupt */
    if( 0ul != (shadowIntr & IPC_INTR_STRUCT_INTR_MASK_RELEASE_Msk))  /* Check for a Release interrupt */
    {
        /* Clear the release interrupt  */
        endpoint->ipcIntrPtr->INTR = (shadowIntr & IPC_INTR_STRUCT_INTR_MASK_RELEASE_Msk);

        if (endpoint->releaseCallbackPtr != NULL)
        {
            endpoint->releaseCallbackPtr();

            /* Clear the pointer after it was called */
            endpoint->releaseCallbackPtr = NULL;
        }
        else
        {
            if (endpoint->defaultReleaseCallbackPtr != NULL)
            {
                endpoint->defaultReleaseCallbackPtr();
            }
        }

        /* Clear the busy flag when release is detected */
        endpoint->busy = CY_IPC_PIPE_ENDPOINT_NOTBUSY;
    }

    (void)Cy_IPC_Drv_GetInterruptStatus(endpoint->ipcIntrPtr);
}

/*******************************************************************************
* Function Name: Cy_IPC_Pipe_EndpointPause
****************************************************************************//**
*
* This function sets the receiver endpoint to paused state.
*
* \param epAddr
* This parameter is the address (or index in the array of endpoint structures)
* that designates the endpoint to pause.
*
*  \return
*    CY_IPC_PIPE_SUCCESS:           Callback registered successfully
*
*******************************************************************************/
cy_en_ipc_pipe_status_t Cy_IPC_Pipe_EndpointPause(uint32_t epAddr)
{
    cy_stc_ipc_pipe_ep_t * endpoint;

    endpoint = &cy_ipc_pipe_epArray[epAddr];

    /* Disable the interrupts */
    NVIC_DisableIRQ(endpoint->pipeIntrSrc);

    return (CY_IPC_PIPE_SUCCESS);
}

/*******************************************************************************
* Function Name: Cy_IPC_Pipe_EndpointResume
****************************************************************************//**
*
* This function sets the receiver endpoint to active state.
*
* \param epAddr
* This parameter is the address (or index in the array of endpoint structures)
* that designates the endpoint to resume.
*
*  \return
*    CY_IPC_PIPE_SUCCESS:           Callback registered successfully
*
*******************************************************************************/
cy_en_ipc_pipe_status_t Cy_IPC_Pipe_EndpointResume(uint32_t epAddr)
{
    cy_stc_ipc_pipe_ep_t * endpoint;

    endpoint = &cy_ipc_pipe_epArray[epAddr];

    /* Enable the interrupts */
    NVIC_EnableIRQ(endpoint->pipeIntrSrc);

    return (CY_IPC_PIPE_SUCCESS);
}


/* [] END OF FILE */
