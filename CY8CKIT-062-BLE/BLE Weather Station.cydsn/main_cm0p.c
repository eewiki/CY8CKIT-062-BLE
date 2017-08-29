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

#include "project.h"
#include "BLE_applications.h"
#include "I2C_1.h"
 
/* This flag is used by application to know whether a Central device has been 
   connected. This is updated in BLE event callback function */ 
extern bool deviceConnected;
 
/*These flags are set when the Central device writes to CCCD of the 
  Sensor Characteristic to enable notifications */
extern bool sendSi7005TempNotifications;
extern bool sendSi7005HumidityNotifications;
 
int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    
    /* Disable the Watchdog Timer to avoid CPU resets */
    Cy_WDT_Unlock();
    Cy_WDT_Disable();    
    
    I2C_1_Start();
    
    /* Start BLE component and register the customEventHandler function. This 
	   function exposes the events from BLE component for application use */
    Cy_BLE_Start(customEventHandler);
    
    for(;;)
    {
        /* Process event callback to handle BLE events. The events generated 
		   and used for this application are inside the 'customEventHandler' 
           routine */
        Cy_BLE_ProcessEvents();
        
        /* If a connection is detected, handle Sensor data transmission */
		if(deviceConnected == true)
		{
			/* Send Si7005 Temperature data when respective notification is 
               enabled */
            if(sendSi7005TempNotifications == CCCD_NOTIFY_BIT_MASK)
			{
				/* Send Si7005 Temperature data when respective notification is 
                   enabled */
                handleSi7005Temp();
			}
            if(sendSi7005HumidityNotifications == CCCD_NOTIFY_BIT_MASK)
			{
				/* Send Si7005 Humidity data when respective notification is 
                   enabled */
				handleSi7005Humidity();
			}                  
		}        
        
        /* Start the BLE advertisement if required */
        startAdvertisement();        
    }
}

/* [] END OF FILE */
