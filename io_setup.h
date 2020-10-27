/* ************************************************************************** */
/** io_setup.h

  @Description
    Setting up IO, PPS, and buttons/LED's.
 
 */
/* ************************************************************************** */

#ifndef _IO_SETUP_H    /* Guard against multiple inclusion */
#define _IO_SETUP_H

#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/attribs.h>

#define LED1        LATDbits.LATD0
#define LED2        LATDbits.LATD1
#define LED3        LATDbits.LATD2
#define BUTTON1     PORTDbits.RD6
#define BUTTON2     PORTDbits.RD7
#define BUTTON3     PORTDbits.RD13

void system_reg_unlock(void);
void system_reg_lock(void);
void io_setup(void);
void delay(int ms);                 // *only edit if clock changes* Delay function in ms   
void buttons(void);                 // *DO NOT EDIT* Polls buttons and calls button_off/on
void uart1_setup(void);

#endif /* _IO_SETUP_H */

/* *****************************************************************************
 End of File
 */
