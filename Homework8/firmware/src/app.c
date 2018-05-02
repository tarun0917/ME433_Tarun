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
#include<stdio.h>
#include "i2c_master_noint.h"
#include"ST7735.h"

#define TIME 48000000
#define SLAVE_ADDRESS 0x6b
#define length 14

unsigned char name = 0;
unsigned char data[14];
char message[30];
int i,j;
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
void initgyro(void)
{
    i2c_master_setup();
    i2c_master_start();
    i2c_master_send(SLAVE_ADDRESS<<1);
    i2c_master_send(0x10);
    i2c_master_send(0x82);
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(SLAVE_ADDRESS<<1);
    i2c_master_send(0x11);
    i2c_master_send(0x88);
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(SLAVE_ADDRESS<<1);
    i2c_master_send(0x12);
    i2c_master_send(0x04);
    i2c_master_stop();
}

unsigned char whoami(void)
{
    unsigned char value;
    i2c_master_start();
    i2c_master_send((SLAVE_ADDRESS << 1));        // hardware address and write bit
    i2c_master_send(0x0F);                     // PORT register = 0x09
    i2c_master_restart();                      // send a RESTART so we can begin reading 
    i2c_master_send((SLAVE_ADDRESS << 1) | 1); // send slave address, left shifted by 1,
                                              // and then a 1 in lsb, indicating read
    value = i2c_master_recv();                // receive a byte from the bus
    i2c_master_ack(1);                      // send NACK (1):  master needs no more bytes
    i2c_master_stop(); 
    return value;
}

void I2C_read_multiple(unsigned char * data) { 
    int i;
    i2c_master_start();
    i2c_master_send((SLAVE_ADDRESS << 1));
    i2c_master_send(0x20);
    i2c_master_restart(); 
    i2c_master_send((SLAVE_ADDRESS << 1) | 1); 
    for (i = 0; i < length; i++) 
       {
        data[i] = i2c_master_recv(); 
        if (i==13) 
            i2c_master_ack(1);
        else 
            i2c_master_ack(0); 
        }
    i2c_master_stop();
}

void drawx(short x, short y, float value, short startcolour, short endcolour)  //function to draw the progress bar line by line
{
    int i,j;
    
    if(value<0)
    {
        for(i=0;i<50;i++)
            if(i<-(value))
                for(j=0;j<4;j++)
                    LCD_drawPixel(x+i,y+j,startcolour);
            else
                for(j=0;j<4;j++)
                    LCD_drawPixel(x+i,y+j,endcolour);
       
        for(i=0;i<50;i++)
            for(j=0;j<4;j++)
                LCD_drawPixel(x-i+4,y+j,endcolour);
    }
    else if(value>0)
    {
        for(i=0;i<50;i++)
            if(i<value)
                for(j=0;j<4;j++)
                    LCD_drawPixel(x-i,y+j,startcolour);
            else
                for(j=0;j<4;j++)
                    LCD_drawPixel(x-i,y+j,endcolour);
            
          for(i=0;i<50;i++)
            for(j=0;j<4;j++)
                LCD_drawPixel(x+i,y+j,endcolour);
    }
       
       
}

void drawy(short x, short y, float value, short startcolour, short endcolour)  //function to draw the progress bar line by line
{
    int i,j;
    
    if(value<0)
    {
        for(i=0;i<50;i++)
            if(i<-(value))
                for(j=0;j<4;j++)
                    LCD_drawPixel(x+j,y+i,startcolour);
            else
                for(j=0;j<4;j++)
                    LCD_drawPixel(x+j,y+i,endcolour);
            
         for(i=0;i<50;i++)
            for(j=0;j<4;j++)
                LCD_drawPixel(x+j,y-i,endcolour);
    }
    else if(value>0)
    {
        for(i=0;i<50;i++)
            if(i<value)
                for(j=0;j<4;j++)
                    LCD_drawPixel(x+j,y-i,startcolour);
            else
                for(j=0;j<4;j++)
                    LCD_drawPixel(x+j,y-i,endcolour);
            
          for(i=0;i<50;i++)
            for(j=0;j<4;j++)
                LCD_drawPixel(x+j,y+i,endcolour);
    }
       
       
}


void LCD_drawChar(short x, short y, char character,short txtcolour, short bckcolour)
{
    int i=0,j=0;
    for(i=0;i<5;i++)                                               // printing a 8x5 pixel which represents a character
        for(j=0;j<8;j++)
            if((ASCII[character-0x20][i]>>j)&1)                    // And operation with 1 to check if a value exists there or not 
                LCD_drawPixel(x+i,y+j,txtcolour);                  // To draw the pixel on the screen
            else
                LCD_drawPixel(x+i,y+j,bckcolour);
}

void LCD_drawString(short x, short y, char* message,short txtcolour, short bckcolour)  // function draws the entire string 
{
    int i=0;
    while(message[i])
    {
        LCD_drawChar((x+5*i),y,message[i],txtcolour,bckcolour);    
        i++;
    }
}


void delay(void)
{
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT() < TIME/100){
    }
}

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
    
    
    
     __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 =1 ;
    
    

    initgyro();
    SPI1_init();
    LCD_init();
    __builtin_enable_interrupts();

   
    _CP0_SET_COUNT(0);
    LCD_clearScreen(BLACK);
    name = whoami();
    
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
                appData.state = APP_STATE_SERVICE_TASKS;
                
                
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            
                    I2C_read_multiple(data);
              signed short temp = (data[1] << 8) | data[0]; //16-bit short
              signed short gyX = (data[3] << 8) | data[2];
              signed short gyY = (data[5] << 8) | data[4];
              signed short gyZ = (data[7] << 8) | data[6];
              signed short accX = (data[9] << 8) | data[8];
              signed short accY = (data[11] << 8) | data[10];
              signed short accZ = (data[13] << 8) | data[12];


              for(i=0;i<4;i++)
                  for(j=0;j<4;j++)
                      LCD_drawPixel(60+i,100+j,WHITE);

              float finaccX = accX*0.0061; 
              float finaccY = accY*0.0061;
              sprintf(message,"ACC_X = %d  ",accX);
              LCD_drawString(5, 14, message, WHITE,BLACK); 
              sprintf(message,"ACC_Y = %d   ",accY);
              LCD_drawString(5, 23, message, WHITE,BLACK); 



              drawx(60,100,finaccX,WHITE,BLACK);
              drawy(60,100,finaccY,WHITE,BLACK);

              if(name==105)
                {
                  sprintf(message,"WHO_AM_I = %d",name);
                  LCD_drawString(5, 5, message, WHITE,BLACK); 
                  LATAINV = 0x0010;
                }

                    break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
