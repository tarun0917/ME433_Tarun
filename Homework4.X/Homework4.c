#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>


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
#define CS LATAbits.LATA0       // chip select pin

static volatile float trianglewave[200];
static volatile float sinwave[200];

    
    
    
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void setVoltage (char a, float v){
    unsigned short t ;
    int temp = v;

    //t = t & (v<<12);
    CS = 0;
    t = (a<<15|0b0111000000000000) | (temp<<2);
    spi_io((t & 0xFF00) >> 8 ); // most significant byte of address
    spi_io(t & 0x00FF);         // the least significant address byte
    CS = 1;
}

void initSPI1(void)
{
  TRISAbits.TRISA0 = 0;     // set the pin A0 as a outpin pin for Chip Select
  CS = 1;                   // set the value of the chip select pin as high, when cs is high, it is not active    
  RPA1Rbits.RPA1R = 0b0011; // set the pin A1 as a SDO pin
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x1;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 1
  
 }


void waves(){
  
    int i =0;
    int value =0;
    for(i = 0; i < 100; i++) {
		sinwave[i] = 512 + 512 * sin(2 * 3.14 * 10 * (i % 100) / 1000);  // to generate the sin wave
        if(i<50)
        {trianglewave[i] = i * 1024/50;
        value = i * 1024/50;}
        else 
        trianglewave[i] = value -  (i * 1024/50);}
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
    initSPI1();
    __builtin_enable_interrupts();

    _CP0_SET_COUNT(0);

    waves();
   setVoltage(0, 0);
   setVoltage(1, 0);
    while(1) {
        static int count = 0;
        if (_CP0_GET_COUNT() >= TIME/2/1000) {
            _CP0_SET_COUNT(0);
            setVoltage(0, sinwave[count]);
            setVoltage(1, trianglewave[count]);
            count++;
            if (count > 99)
                count = 0;
        }
    }
    return 0;
}

