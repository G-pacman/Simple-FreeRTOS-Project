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
#include "queue.h"
#include "x500_accelerometer_spi.h"
#include "io_setup.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
/* The rate at which data is sent to the queue.  The 200ms value is converted
to ticks using the portTICK_RATE_MS constant. */
#define QUEUE_SEND_FREQUENCY_MS         ( 200 / portTICK_PERIOD_MS )
/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define QUEUE_LENGTH                    ( 1 )
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
/* The queue used by both tasks. */
QueueHandle_t xQueue1 = NULL;
QueueHandle_t xQueue2 = NULL;
//const unsigned long ulValueToSend1 = 100UL;
//const unsigned long ulValueToSend2 = 1000UL;

//const unsigned long ulValueToSend1 = 'a';
//const unsigned long ulValueToSend2 = 'b';

float ulValueToSend1 = 0;
float ulValueToSend2 = 0;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
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
   /* Create the queue. */
   xQueue1 = xQueueCreate( 10, sizeof( unsigned long ) );
   xQueue2 = xQueueCreate( 10, sizeof( unsigned long ) );
   io_setup();
   uart1_setup();
   spi2_setup();
   accel_setup();
}

void RTOS_SPI2_READ_X(void)
{
   int16_t X_H; 
   int16_t X_L; 
   int16_t X;
   
   X_H = spi2_read_register(0x29);
   X_L = spi2_read_register(0x28);
   
   X = X_H << 8; // Combine data from both registers
   X = X | X_L; // See Lab2 manual for instructions
   
   //printf("%f", 0.000061f * X);
   ulValueToSend1 = (float)(0.000061f * X);
   
}

void RTOS_SPI2_READ_Y(void)
{
    int16_t Y_H; 
   int16_t Y_L; 
   int16_t Y;
   
    Y_H = spi2_read_register(0x2B);
    Y_L = spi2_read_register(0x2A);
    
   Y = Y_H << 8;// Combine data from both registers
   Y = Y | Y_L;// See Lab2 manual for instructions

   //printf("%f", 0.000061f * Y);
   ulValueToSend2 = (float)(0.000061f * Y);
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    
    RTOS_SPI2_READ_X();
    RTOS_SPI2_READ_Y();
   /* Send to the queue - causing the queue receive task2 to unblock and
   toggle the LED.  0 is used as the block time so the sending operation
   will not block - it shouldn't need to block as the queue should always
   be empty at this point in the code. */
   //xQueueSend( ??, ??, 0U );
   xQueueSend( xQueue1, &ulValueToSend1, 0U );
   /* Send to the queue - causing the queue receive task1 to unblock and
   toggle the LED.  0 is used as the block time so the sending operation
   will not block - it shouldn't need to block as the queue should always
   be empty at this point in the code. */
   
   xQueueSend( xQueue2, &ulValueToSend2, 0U );

   /* Place this task in the blocked state until it is time to run again.
	The block time is specified in ticks, the constant used converts ticks
   to ms.  While in the Blocked state this task will not consume any CPU
   time. */
   vTaskDelay(QUEUE_SEND_FREQUENCY_MS );
}
 

/*******************************************************************************
 End of File
 */
