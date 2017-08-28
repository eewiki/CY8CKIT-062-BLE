/*****************************************************************************
* MIT License
* 
* Copyright (c) 2017 Robert Nelson <robert.nelson@digikey.com>
*  
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*  
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*  
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*****************************************************************************/
/*****************************************************************************
* Based off: CE218134_BLE_CapSense.pdf
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and 
* foreign), United States copyright laws and international treaty provisions. 
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the 
* Cypress source code and derivative works for the sole purpose of creating 
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited 
* without the express written permission of Cypress.
* 
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products 
* where a malfunction or failure may reasonably be expected to result in 
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's 
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*****************************************************************************/

/* Header file includes */
#include "BLE_applications.h"

/* Client Characteristic Configuration descriptor (CCCD) data length */
#define CCCD_DATA_LEN   (uint8_t) (0x02u)

/* Indexes of a two-byte CCCD array */
#define CCCD_INDEX_0    (uint8_t) (0x00u)
#define CCCD_INDEX_1    (uint8_t) (0x01u)

/* Null byte used to initialize CCCD values */
#define CCCD_NULL       (uint8_t) (0x00u)

/* 'connectionHandle' stores the BLE connection parameters */
cy_stc_ble_conn_handle_t connectionHandle;


/* This flag is used by application to know whether a Central device has been 
   connected. This value is continuously updated in BLE event callback
   function */
bool                deviceConnected = false;

/* This is used to restart advertisements in the main firmware loop */
bool                restartAdvertisement = true;

/* Status flag for the Stack Busy state. This flag is used to notify the 
   application if there is stack buffer free to push more data or not */
bool                busyStatus = false;

/*******************************************************************************
* Function Name: void startAdvertisement(void)
********************************************************************************
* Summary:
*  Check the "restartAdvertisement" flag and start the BLE advertisement if
*  the flag is "true"
*
* Parameters:
*  Void
*
* Return:
*  void
*
*******************************************************************************/
void startAdvertisement(void)
{
    /* Check if the restartAdvertisement flag is set by the event handler */
    if(restartAdvertisement)
	{
		/* Reset 'restartAdvertisement' flag */
		restartAdvertisement = false;
		
		/* Start Advertisement and enter discoverable mode */
		Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,
                       CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);	
	}
}

/*******************************************************************************
* Function Name: void customEventHandler(uint32_t event, void *eventParameter)
********************************************************************************
* Summary:
*  Call back event function to handle various events from the BLE stack
*
* Parameters:
*  event            :	event returned
*  eventParameter   :	link to value of the events returned
*
* Return:
*  void
*
*******************************************************************************/
void customEventHandler(uint32_t event, void *eventParameter)
{
    /* Local variable to store the data received as part of the write request
       events */
    cy_stc_ble_gatts_write_cmd_req_param_t   *writeReqParameter;
    
    /* Take an action based on the current event */
    switch (event)
    {
        /* This event is received when the BLE stack is Started */
        case CY_BLE_EVT_STACK_ON:
            
            /* Set restartAdvertisement flag to allow calling Advertisement
               API from the main function */
            restartAdvertisement = true;
            break;

        /* ~~~~~~~~~~~~~~~~~~~~~~GAP EVENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
            
        /* If the current BLE state is Disconnected, then the Advertisement
           Start-Stop event implies that advertisement has stopped */
        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
            
            /* Check if the advertisement has stopped */
            if (Cy_BLE_GetState() == CY_BLE_STATE_STOPPED)
            {
                /* Set restartAdvertisement flag to allow calling Advertisement
                   API from the main function */
                restartAdvertisement = true;
            }
            break;
        
        /* This event is received when device is disconnected */
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            
            /* Set restartAdvertisement flag to allow calling Advertisement
             API from the main function */
            restartAdvertisement = true;
            break;

        /* ~~~~~~~~~~~~~~~~~~~~~~GATT EVENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        
        /* This event is received when device is connected over GATT level */    
        case CY_BLE_EVT_GATT_CONNECT_IND:
            
            /* Update attribute handle on GATT Connection*/
            connectionHandle = *(cy_stc_ble_conn_handle_t *) eventParameter;

            /* This flag is used by the application to check the connection
               status */
            deviceConnected = true;
            break;
        
        /* This event is received when device is disconnected */
        case CY_BLE_EVT_GATT_DISCONNECT_IND:
           
            /* Update deviceConnected flag*/
            deviceConnected = false;
            
            /* Call the functions that handle the disconnect events for all 
               custom services */
            //FIXME: handleDisconnectEvent_X();
            break;
        
        /* This event is received when Central device sends a Write command
           on an Attribute */
        case CY_BLE_EVT_GATTS_WRITE_REQ:
            
            /* Read the write request parameter */
            writeReqParameter = (cy_stc_ble_gatts_write_cmd_req_param_t *) 
                                eventParameter;

            /* When this event is triggered, the peripheral has received a 
               write command on the custom  characteristic. Check if command
               fits any of the custom attributes and update the flag for
               sending notifications by the respective service */
            //FIXME: handleWriteRequestX();
            /* Send the response to the write request received. */
            Cy_BLE_GATTS_WriteRsp(connectionHandle);
            break;
        
        /* This event is generated when the internal stack buffer is full and no
           more data can be accepted or the stack has buffer available and can 
           accept data. This event is used by application to prevent pushing lot
           of data to the BLE stack. */
        case CY_BLE_EVT_STACK_BUSY_STATUS:
            
            /* Extract the present stack status */
            busyStatus = *(uint8_t *) eventParameter;
            break;
        
        /* Do nothing for all other events */
        default:
            break;
    }
}

/* [] END OF FILE */
