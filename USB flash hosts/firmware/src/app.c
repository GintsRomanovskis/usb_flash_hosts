/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
SYS_TMR_HANDLE handle20ms, handle30ms;
const uint8_t writeData[12]  __attribute__((aligned(16))) = "Hello World ";

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void Test_Callback ( uintptr_t context, uint32_t currTick );


void Test_Callback ( uintptr_t context, uint32_t currTick )
{
    if ( context == 1 )
    {
        BSP_LED_1Toggle();
    }
    else
    {
        BSP_LED_2Toggle();
    }
}

/* This code shows usage of the USB_HOST_MSD_Transfer function. The SCSI Block
 * Driver Library uses this function to send a SCSI Inquiry Command to the
 * device. Note how the commandCompleted flag in the SCSI instance object
 * tracks the completion of the transfer. This flag is updated in the transfer
 * callback. */

/*void _USB_HOST_SCSI_TransferCallback
(
    USB_HOST_MSD_LUN_HANDLE lunHandle,
    USB_HOST_MSD_TRANSFER_HANDLE transferHandle,
    USB_HOST_MSD_RESULT result,
    size_t size,
    uintptr_t context
)
{
    int scsiObjIndex;
    USB_HOST_SCSI_OBJ * scsiObj;
    USB_HOST_SCSI_COMMAND_OBJ * commandObj;
    USB_HOST_SCSI_EVENT event;

    // Get the SCSI object index from the lunHandle 
    scsiObjIndex = _USB_HOST_SCSI_LUNHandleToSCSIInstance(lunHandle);

    // Get the pointer to the SCSI object 
    scsiObj = &gUSBHostSCSIObj[scsiObjIndex];

     //Pointer to the command object 
    commandObj = &scsiObj->commandObj;

    // The processed size 
    commandObj->size = size;

    // The result of the command 
    commandObj->result = result;

    // Let the main state machine know that the command is completed 
    commandObj->commandCompleted = true;

     //The rest of code is not shown here for the sake of brevity 
}

*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    appData.deviceIsConnected = false;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}
//eventhandleris - atkariibaa no staavokla
//atziimee vai failsisteemai ir piemonteeta vai nee. . 
void APP_SYSFSEventHandler(SYS_FS_EVENT event, void * eventData, uintptr_t context)
{
    switch(event)
    {
        case SYS_FS_EVENT_MOUNT:
            appData.deviceIsConnected = true;
            break;
            
        case SYS_FS_EVENT_UNMOUNT:
            appData.deviceIsConnected = false;
            
            break;
            
        default:
            break;
    }
}
//usb host eventhandler
USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler (USB_HOST_EVENT event, void * eventData, uintptr_t context)
{
    switch (event)
    {
        case USB_HOST_EVENT_DEVICE_UNSUPPORTED:
            break;
        default:
            break;
                    
    }
    
    return(USB_HOST_EVENT_RESPONSE_NONE);
}



/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
        //LED tiek sl?gti sekojoshi:
        
        //LED1 - On - Pievienota ieriice
        //LED2 - On - Gaidiishanas rezhiims 
        //LED3 - On - ERROR
        //Visi LED - Off - nav piesleegts USB Flash
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
           // handle20ms = SYS_TMR_CallbackPeriodic ( 1000, 1, Test_Callback );
           // handle30ms = SYS_TMR_CallbackPeriodic ( 2000, 2, Test_Callback );
            //izsleedz visus LED
            BSP_LED_1Off();
            BSP_LED_2Off();
            BSP_LED_3Off();
            
            //inicializee event handleri.
            SYS_FS_EventHandlerSet(APP_SYSFSEventHandler, (uintptr_t)NULL);
            if (appInitialized)
            {
                //pagaidaam sheit lai ir shaadi
                appData.state = APP_STATE_USB;
            }
            break;
        }//inicializee USB hostu 
        case APP_STATE_USB:
            USB_HOST_EventHandlerSet(APP_USBHostEventHandler, 0);
            USB_HOST_BusEnable(0);
               if(USB_HOST_BusIsEnabled(0))
            {
                appData.state = APP_WAIT_FOR_DEVICE_ATTACH;
            }
            break;
            //gaida liidz tiks pievienota ieriice
        case APP_WAIT_FOR_DEVICE_ATTACH:

         //kad ieriice pievienota, paarsleedzas uz naakamo staavokli APP_OPEN_FILE
            if(appData.deviceIsConnected)
            {
                BSP_LED_1On();//iesleedz 1.LED
                BSP_LED_2Off();//izsleedz 2.diodi. 
                appData.state = APP_OPEN_FILE;
            }

            break;
            
        //
        //sheit naaks viss kas saistiits ar usb pievienoshanu. 
        //
        //atveram failu, kuru rakstiit uz USB. Pagaidaam bez usb. 
        case APP_OPEN_FILE:
           
              appData.fileHandle = SYS_FS_FileOpen("/mnt/myDrive1/simpleText.txt",(SYS_FS_FILE_OPEN_APPEND_PLUS));
            if(appData.fileHandle == SYS_FS_HANDLE_INVALID)
            {
                //ja failu nevar atveert , atgriezh kljuudu
                appData.state = APP_ERROR;
            }
            else
            {
              
                    //ja failu var atveert, paariet uz rakstiishanu
                appData.state = APP_STATE_WRITE_TO_FILE;

            }
            break;
//raksta failu uz failsisteemu. 
        case APP_STATE_WRITE_TO_FILE:

            
            if (SYS_FS_FileWrite( appData.fileHandle, (const void *) writeData, 12 ) == -1)
            {
                //neizdodas rakstiit uz failu, atgriezh kljuudu
                SYS_FS_FileClose(appData.fileHandle);
                appData.state = APP_ERROR;

            }
            else
            {
                //ja izdodads ierakstiit, aizver failu
                appData.state = APP_STATE_CLOSE_FILE;
            }

            break;
//aizver failu 
        case APP_STATE_CLOSE_FILE:

            
            SYS_FS_FileClose(appData.fileHandle);

            //paarsleedzas uz IDLE
            appData.state = APP_STATE_END_IDLE;
            break;
            //gaidiishanas rezhiims
        case APP_STATE_END_IDLE:
            printf("Success! ");
            BSP_LED_2On();
            BSP_LED_1Off();
            if(appData.deviceIsConnected == false)
            {
                appData.state = APP_WAIT_FOR_DEVICE_ATTACH;
                BSP_LED_2Off();
            }
            break;       
        
        case APP_ERROR:
            
            // kljuudas gadiijumaaIesleedz tresho LED uz plates. 
            BSP_LED_3On();
            break;
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
//zemaak esoshais kods nepilniigs un visticamaak pilniigi nevajadziigs. 

/*void USB_HOST_SCSI_Tasks(USB_HOST_MSD_LUN_HANDLE lunHandle)
{
    switch(scsiObj->state)
    {
       // For the sake of brevity, only one SCSI command is show here 
        case USB_HOST_SCSI_STATE_INQUIRY_RESPONSE:

            // We get the SCSI Enquiry response. Although there isn't much
            // that we can do with this data 
            _USB_HOST_SCSI_InquiryResponseCommand(scsiObj->commandObj.cdb);

            // The commandCompleted flag will be updated in the callback.
             //Update the state and send the command.   
            scsiObj->commandObj.inUse = true;
            scsiObj->commandObj.commandCompleted = false;
            scsiObj->commandObj.generateEvent = false;

            result = USB_HOST_MSD_Transfer(scsiObj->lunHandle,
                    scsiObj->commandObj.cdb, 6, scsiObj->buffer, 36,
                    USB_HOST_MSD_TRANSFER_DIRECTION_DEVICE_TO_HOST,
                    _USB_HOST_SCSI_TransferCallback, (uintptr_t)(scsiObj));

            if(result == USB_HOST_MSD_RESULT_SUCCESS)
            {
                scsiObj->state = USB_HOST_SCSI_STATE_WAIT_INQUIRY_RESPONSE;
            }

            break;

        default:
            break;

    }
}  */ 

/*******************************************************************************
 End of File
 */
