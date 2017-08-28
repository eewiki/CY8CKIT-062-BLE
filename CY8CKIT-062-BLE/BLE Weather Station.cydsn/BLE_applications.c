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
 
/* Read and write length of the si7005 data */
#define SI7005_DATA_LEN    (uint8_t) (0x02u)
 
/* Respective indexes of the si7005 */
#define SI7005_TEMP0_INDEX   (uint8_t) (0x00u)
#define SI7005_TEMP1_INDEX   (uint8_t) (0x01u)
 
/* Data lengths of si7005 Temperature data sent over BLE notifications */
#define SI7005_TEMP    (uint8_t) (0x01u)
#define SI7005_HUMIDITY    (uint8_t) (0x01u)
 
/* Array to store the present si7005 Temperature data. */
uint8_t             Si7005TempDataArray[SI7005_DATA_LEN];
 
/* These flags are set when the Central device writes to CCCD (Client 
   Characteristic Configuration Descriptor) of the si7005 Temperature 
   Characteristic to enable notifications */
bool                sendSi7005TempNotifications = false;
 
/* This flag is used by application to know whether a Central device has been 
   connected. This value is continuously updated in BLE event callback
   function */
bool                deviceConnected = false;
 
/* This is used to restart advertisements in the main firmware loop */
bool                restartAdvertisement = true;
 
/* Status flag for the Stack Busy state. This flag is used to notify the 
   application if there is stack buffer free to push more data or not */
bool                busyStatus = false;
 
#include "I2C_1.h"
 
#define I2C_TIMEOUT 100
 
#define SI7005_ADDR 0x40
#define SI7005_DEV_ID 0x50
 
/* SI7005 Registers */
#define SI7005_REG_STATUS         0x00
#define SI7005_REG_DATA           0x01
#define SI7005_REG_CONFIG         0x03
#define SI7005_REG_ID             0x11
 
/* Status Register */
#define SI7005_STATUS_NOT_READY   0x01
 
/* Config Register */
#define SI7005_CONFIG_START       0x01
#define SI7005_CONFIG_HEAT        0x02
#define SI7005_CONFIG_HUMIDITY    0x00
#define SI7005_CONFIG_HUMIDITY_START    0x01
#define SI7005_CONFIG_TEMPERATURE 0x10
#define SI7005_CONFIG_TEMPERATURE_START 0x11
#define SI7005_CONFIG_FAST        0x20
 
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
* Function Name: void sendSi7005TempNotification(uint8_t *Si7005TempDataArray)
********************************************************************************
* Summary:
*  Send Si7005Temp data as BLE Notifications. This function updates
*  the notification handle with data and triggers the BLE component to send
*  notification
*
* Parameters:
*  Si7005TempDataArray:	 Si7005Temp value
*
* Return:
*  void
*
*******************************************************************************/
void sendSi7005TempNotification(uint8_t *Si7005TempDataArray)
{
    cy_stc_ble_gatts_handle_value_ntf_t  Si7005TempnotificationHandle;
 
    /* If stack is not busy, then send the notification */
    if (busyStatus == CY_BLE_STACK_STATE_FREE)
    {
        /* Update notification handle with Si7005 Temperature data */
        Si7005TempnotificationHandle.connHandle = connectionHandle;
        Si7005TempnotificationHandle.handleValPair.attrHandle = 
                            CY_BLE_WEATHER_STATION_TEMPERATURE_CHAR_HANDLE;
        Si7005TempnotificationHandle.handleValPair.value.val = 
                            Si7005TempDataArray;
        Si7005TempnotificationHandle.handleValPair.value.len =
                            SI7005_DATA_LEN;
 
        /* Send the updated handle as part of attribute for notifications */
        Cy_BLE_GATTS_Notification(&Si7005TempnotificationHandle);
    }
}
 
/*******************************************************************************
* Function Name: void handleSi7005Temp(void)
********************************************************************************
* Summary:
*  Read Si7005 Tempearature data
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handleSi7005Temp(void)
{
    /* Si7005 Temperature value from the previous scan */
    static uint16_t   previousSi7005Temp;
 
    /*  Si7005 Temperature value from the current scan */
    uint16_t          currentSi7005Temp;
    int16_t           broadcastSi7005Temp;    
    float             Si7005Temp;
 
    static uint8_t tmp0 = 0;
    static uint8_t tmp1 = 0;    
 
    I2C_1_MasterSendStart(SI7005_ADDR, CY_SCB_I2C_WRITE_XFER, I2C_TIMEOUT);
    I2C_1_MasterWriteByte(SI7005_REG_CONFIG, I2C_TIMEOUT);
    I2C_1_MasterWriteByte(SI7005_CONFIG_TEMPERATURE_START, I2C_TIMEOUT);
    I2C_1_MasterSendStop(I2C_TIMEOUT);
 
    CyDelay(100);
 
    I2C_1_MasterSendStart(SI7005_ADDR, CY_SCB_I2C_WRITE_XFER, I2C_TIMEOUT);
    I2C_1_MasterWriteByte(SI7005_REG_DATA, I2C_TIMEOUT);        
    I2C_1_MasterSendReStart(SI7005_ADDR, CY_SCB_I2C_READ_XFER, I2C_TIMEOUT);
    I2C_1_MasterReadByte(CY_SCB_I2C_ACK, &tmp0, I2C_TIMEOUT);
    I2C_1_MasterReadByte(CY_SCB_I2C_NAK, &tmp1, I2C_TIMEOUT);        
    I2C_1_MasterSendStop(I2C_TIMEOUT);
    currentSi7005Temp = (tmp0 << 8);
    currentSi7005Temp = (currentSi7005Temp | (tmp1 & 0xFF));
    currentSi7005Temp = (currentSi7005Temp >> 2);
 
    //org.bluetooth.characteristic.temperature sint16, Celsius with 0.01 degress resolution.
    Si7005Temp = (float)(currentSi7005Temp * 100.0);
    Si7005Temp = (float)(Si7005Temp / 32);
    Si7005Temp = (float)(Si7005Temp - 5000);
    broadcastSi7005Temp = (int16_t)Si7005Temp;
 
    //org.bluetooth.characteristic.temperature is in little Endian..
    Si7005TempDataArray[SI7005_TEMP1_INDEX] = (int8_t)((broadcastSi7005Temp & 0xFF00) >> 8);
    Si7005TempDataArray[SI7005_TEMP0_INDEX] = (int8_t)(broadcastSi7005Temp & 0x00FF);
 
    if (currentSi7005Temp != previousSi7005Temp)
    {    
        /* Send data over Si7005 Temperature notification */
        sendSi7005TempNotification(Si7005TempDataArray);
 
        /* Update the local static variable with the present finger
		    position on the slider */
        previousSi7005Temp = currentSi7005Temp;
    };
}
 
/*******************************************************************************
* Function Name: void handleWriteRequestforSi7005Temp
*                     (cy_stc_ble_gatts_write_cmd_req_param_t *writeRequest)
********************************************************************************
* Summary:
*  This functions handles the 'write request' event for the Si7005Temp service
*
* Parameters:
*  writeRequest : pointer to the write request parameters from the central       
*
* Return:
*  void
*
*******************************************************************************/
void handleWriteRequestforSi7005Temp(cy_stc_ble_gatts_write_cmd_req_param_t *writeRequest)
{
    /* Local variable 'attributeHandle' stores attribute parameters*/
    cy_stc_ble_gatts_db_attr_val_info_t  attributeHandle;
 
    /* Handle value to update the CCCD */
    cy_stc_ble_gatt_handle_value_pair_t  Si7005TempNotificationCCCDhandle;
 
    /* Local variable to store the current CCCD value */
    uint8_t                         Si7005TempCCCDvalue[CCCD_DATA_LEN];
 
    /* Extract the Write value sent by the Client for CapSense Button CCCD */
    if (writeRequest->handleValPair.value.val
        [CY_BLE_WEATHER_STATION_TEMPERATURE_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_INDEX]
         == true)
    {
        sendSi7005TempNotifications = true;
    }
    else if (writeRequest->handleValPair.value.val
             [CY_BLE_WEATHER_STATION_TEMPERATURE_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_INDEX]
             == false)
    {
        sendSi7005TempNotifications = false;
    }
 
    /* Write the present CapSense notification status to the local variable */
    Si7005TempCCCDvalue[CCCD_INDEX_0] = sendSi7005TempNotifications;
    Si7005TempCCCDvalue[CCCD_INDEX_1] = CCCD_NULL;
 
    /* Update CCCD handle with notification status data */
    Si7005TempNotificationCCCDhandle.attrHandle 
    = CY_BLE_WEATHER_STATION_TEMPERATURE_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE;
 
    Si7005TempNotificationCCCDhandle.value.val = Si7005TempCCCDvalue;
    Si7005TempNotificationCCCDhandle.value.len = CCCD_DATA_LEN;
 
    /* Report data to BLE component for sending data when read by the central
       device */
    attributeHandle.handleValuePair = Si7005TempNotificationCCCDhandle;
    attributeHandle.offset = CCCD_NULL;
    attributeHandle.connHandle = connectionHandle;
    attributeHandle.flags = CY_BLE_GATT_DB_PEER_INITIATED;
    Cy_BLE_GATTS_WriteAttributeValueCCCD(&attributeHandle);
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
            handleSi7005Temp();
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
            if (CY_BLE_WEATHER_STATION_TEMPERATURE_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE
                == writeReqParameter->handleValPair.attrHandle)
            {
                handleWriteRequestforSi7005Temp(writeReqParameter);
            }

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
