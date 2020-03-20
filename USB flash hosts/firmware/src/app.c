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

/* '?is strings tiks ierakst?ts file*/
const uint8_t writeData[12] __attribute__((aligned(16))) = "Hello World ";

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
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

void APP_Initialize(void) {
    /* Piefiks? programmas s?kotn?jo st?voli . */
    appData.state = APP_STATE_BUS_ENABLE;
    appData.deviceIsConnected = false;
}

USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler(USB_HOST_EVENT event, void * eventData, uintptr_t context) {
    switch (event) {
        case USB_HOST_EVENT_DEVICE_UNSUPPORTED:
            break;
        default:
            break;

    }

    return (USB_HOST_EVENT_RESPONSE_NONE);
}

//atziimee vai failsisteemas ieriice ir piemonteeta vai nee. . 

void APP_SYSFSEventHandler(SYS_FS_EVENT event, void * eventData, uintptr_t context) {
    switch (event) {
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

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    switch (appData.state) {
        case APP_STATE_BUS_ENABLE:

            /* Iestat?t notikumu apstr?d?t?ju un iesl?dz bus */
            SYS_FS_EventHandlerSet(APP_SYSFSEventHandler, (uintptr_t) NULL);
            USB_HOST_EventHandlerSet(APP_USBHostEventHandler, 0);
            USB_HOST_BusEnable(0);
            appData.state = APP_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE;
            break;

        case APP_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE:
            if (USB_HOST_BusIsEnabled(0)) {
                appData.state = APP_STATE_WAIT_FOR_DEVICE_ATTACH;
            }
            break;

        case APP_STATE_WAIT_FOR_DEVICE_ATTACH:
            /* gaida ier?ces pievieno?anu. St?vok?a  tiks p?rvietots
             * uz n?kamo st?vokli, kad pievieno?anas notikums
             * ir sa?emts. */

            if (appData.deviceIsConnected) {
                BSP_LED_1On();
                BSP_LED_2Off();
                appData.state = APP_STATE_DEVICE_CONNECTED;
            }

            break;

        case APP_STATE_DEVICE_CONNECTED:

            /*er?ce tika piesl?gta . M?s varam m??in?t uzst?d?t disku */
            appData.state = APP_STATE_OPEN_FILE;
            break;

        case APP_STATE_OPEN_FILE:

            /* m??iniet atv?rt failu pievieno?anai */

            appData.fileHandle = SYS_FS_FileOpen("/mnt/myDrive1/file.txt", (SYS_FS_FILE_OPEN_APPEND_PLUS));
            if (appData.fileHandle == SYS_FS_HANDLE_INVALID) {
                /* Failu nevar?ja atv?rt tiek uzst?d?ta k?uda*/
                appData.state = APP_STATE_ERROR;

            } else {
                /* Fails tika veiksm?gi atv?ts . Tiek p?riets uz  faila rakst??anu */
                appData.state = APP_STATE_WRITE_TO_FILE;

            }
            break;

        case APP_STATE_WRITE_TO_FILE:

            /* M??iniet rakst?t fail? */
            if (SYS_FS_FileWrite(appData.fileHandle, (const void *) writeData, 12) == -1) {
                /* Rakst??ana nebija veiksm?ga. Aizv?rt failu
                 * un izejo?? k??da.*/
                SYS_FS_FileClose(appData.fileHandle);
                appData.state = APP_STATE_ERROR;

            } else {
                /* Rakst??ana bija veiksm?ga un pabeigta.Fails tiek aizv?rts */
                appData.state = APP_STATE_CLOSE_FILE;
            }

            break;

        case APP_STATE_CLOSE_FILE:

            /* Aizver failu*/
            SYS_FS_FileClose(appData.fileHandle);

            /* P?rbaude bija veiksm?ga. ?auj d?kst?v? */
            appData.state = APP_STATE_IDLE;
            break;

        case APP_STATE_IDLE:

            /* aplik?cija par?d?s ?eit, kad demonstr?cija ir pabeigta
             * veiksm?gi. Tiek par?d?ts INDIKATORS . Gaid?t ier?ces atvieno?anu
             * un, ja tas ir atvienots , pagaidiet, kam?r tas tiks  pievienots. */

            BSP_LED_3Off();
            BSP_LED_2On();
            if (appData.deviceIsConnected == false) {
                appData.state = APP_STATE_WAIT_FOR_DEVICE_ATTACH;
                BSP_LED_2Off();
            }
            break;

        case APP_STATE_ERROR:

            /* Datb?bas tiek novirz?tas uz ?o bloku ja APP_STATE 
             * sada?? izkrita ar k??du.*/

            BSP_LED_1On();
            if (SYS_FS_Unmount("/mnt/myDrive") != 0) {
                /*Disku nevar?ja atvienot. */

                appData.state = APP_STATE_ERROR;
            } else {
                /* Diska atvieno?ana ir sekm?ga */
                appData.state = APP_STATE_WAIT_FOR_DEVICE_ATTACH;

            }
            break;

        default:
            break;

    }
}


/*******************************************************************************
 End of File
 */
