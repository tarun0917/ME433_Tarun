#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<stdio.h>
#include "i2c_master_noint.h"
#include"ST7735.h"


// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define TIME 48000000
#define SLAVE_ADDRESS 0x6b
#define length 14


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

int main() {

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
    
    
    unsigned char name = 0;
    unsigned char data[14];
    char message[30];
    int i,j;
    initgyro();
    SPI1_init();
    LCD_init();
    __builtin_enable_interrupts();

   
    _CP0_SET_COUNT(0);
    LCD_clearScreen(BLACK);
    name = whoami();
    
    

  
    while(1) {
	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
	// remember the core timer runs at half the sysclk
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
      
     
      
   
      
    }
    return 0;
}



