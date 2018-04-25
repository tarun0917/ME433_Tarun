#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>
#include"ST7735.h"
#include<stdio.h>

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

  
    
    
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}






void LCD_drawChar(short x, short y, char character,short txtcolour, short bckcolour)
{
    int i=0,j=0;
    for(i=0;i<5;i++)
        for(j=0;j<8;j++)
            if((ASCII[character-0x20][i]>>j)&1)
                LCD_drawPixel(x+i,y+j,txtcolour);
            else
                LCD_drawPixel(x+i,y+j,bckcolour);
}

void LCD_drawString(short x, short y, char* message,short txtcolour, short bckcolour)
{
    int i=0;
    while(message[i])
    {
        LCD_drawChar((x+5*i),y,message[i],txtcolour,bckcolour);
        i++;
    }
}


void LCD_progressbar(short x, short y, short currentvalue, short percentage, short startcolour, short endcolour)
{
    int i=0;
    for(i=0;i<5;i++)
        if(currentvalue<percentage)
            LCD_drawPixel(x,y+i,startcolour);
        else
            LCD_drawPixel(x,y+i,endcolour);
}


void delay(void)
{
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT() < 48000000/30){
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
    //TRISAbits.TRISA4 = 0;
    //TRISBbits.TRISB4 = 1;
    //LATAbits.LATA4 =1 ;
    SPI1_init();
    LCD_init();
    __builtin_enable_interrupts();
    LCD_clearScreen(BLACK);
    _CP0_SET_COUNT(0);

    char message[30];
    char k;
    float fps = 1;
    
    while(1) 
    {
     
        for(k=0;k<101;k++)
        {
         _CP0_SET_COUNT(0);
         sprintf(message,"Hello World! %d",k);
         LCD_drawString(28, 32, message, WHITE,BLACK); 
         
         LCD_progressbar(k+12,70,k,80,BLUE,RED);
         
         sprintf(message,"FPS: %5.2f",fps);
         LCD_drawString(28, 90, message, WHITE,BLACK); 
         fps = 24000000.0/_CP0_GET_COUNT();
         delay();
        }
        LCD_clearScreen(BLACK);
    }
    return 0;
}

