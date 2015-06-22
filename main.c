/*
 * File:   main.c
 * Authors: Sangam Singh, Benjamen Hartley, Dimitris Firfilionis
 *
 * Created on 19 March 2015, 11:27
 */

// Header Files
#include <xc.h>
#include <p18cxxx.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <plib/pconfig.h>
#include <plib/adc.h>
#include <delays.h>

#define USE_AND_MASKS

// Define Analog and Digital pins
#define ADC_3ANA        0b00001100  // analog: AN0->2   digital: AN3->15

// Address locations for I2C - Barometric sensor
#define BAR_WRITE       0b11101110    // Sensor address and Write bit
#define BAR_READ        0b11101111    // Sensor address and Read bit

#define RESET_CMD       0b00011110  // Reset
#define PRESS_CONV_CMD  0b01001000  // Start conversion D1 (Pressure)
#define TEMP_CONV_CMD   0b01011000  // Start conversion D2 (Temperature)
#define READ_ADC_CMD    0b00000000  // Read ADC command


// Address locations for I2C - Relative Humidity and Temperature sensor
#define TEMP_WRITE      0b10000000  // Sensor address and Write bit
#define TEMP_READ       0b10000001  // Sensor address and Read bit

#define TEMP_MEAS       0b11100011  // Hold Master
#define R_HUM_MEAS      0b11100101  // Hold Master
#define W_USER_REG      0b11100110  // Write user register
#define R_USED_REG      0b11100111  // Read user register
#define SOFT_RESET      0b11111110  // Soft Reset

// PIC18F25K80 Configuration Bit Settings

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1L
#pragma config RETEN = ON           // VREG Sleep Enable bit (Ultra low-power regulator is Enabled (Controlled by SRETEN bit))
#pragma config INTOSCSEL = HIGH     // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = HIGH       // SOSC Power Selection and mode Configuration bits (High Power SOSC circuit selected)
#pragma config XINST = OFF          // Extended Instruction Set (Disabled)

// CONFIG1H
#pragma config FOSC = INTIO1        // Oscillator (Internal RC oscillator, CLKOUT function on OSC2)
#pragma config PLLCFG = ON          // PLL x4 Enable bit (Enabled)
#pragma config FCMEN = OFF          // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = OFF           // Internal External Oscillator Switch Over Mode (Disabled)

// CONFIG2L
#pragma config PWRTEN = OFF         // Power Up Timer (Disabled)
#pragma config BOREN = OFF          // Brown Out Detect (Disabled in hardware, SBOREN disabled)
#pragma config BORV = 3             // Brown-out Reset Voltage bits (1.8V)
#pragma config BORPWR = ZPBORMV     // BORMV Power level (ZPBORMV instead of BORMV is selected)

// CONFIG2H
#pragma config WDTEN = OFF          // Watchdog Timer (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WDTPS = 1048576      // Watchdog Postscaler (1:1048576)

// CONFIG3H
#pragma config CANMX = PORTB        // ECAN Mux bit (ECAN TX and RX pins are located on RB2 and RB3, respectively)
#pragma config MSSPMSK = MSK7       // MSSP address masking (7 Bit address masking mode)
#pragma config MCLRE = ON           // Master Clear Enable (MCLR Enabled, RE3 Disabled)

// CONFIG4L
#pragma config STVREN = ON          // Stack Overflow Reset (Enabled)
#pragma config BBSIZ = BB2K         // Boot Block Size (2K word Boot Block size)

// CONFIG5L
#pragma config CP0 = OFF            // Code Protect 00800-01FFF (Disabled)
#pragma config CP1 = OFF            // Code Protect 02000-03FFF (Disabled)
#pragma config CP2 = OFF            // Code Protect 04000-05FFF (Disabled)
#pragma config CP3 = OFF            // Code Protect 06000-07FFF (Disabled)

// CONFIG5H
#pragma config CPB = OFF            // Code Protect Boot (Disabled)
#pragma config CPD = OFF            // Data EE Read Protect (Disabled)

// CONFIG6L
#pragma config WRT0 = OFF           // Table Write Protect 00800-03FFF (Disabled)
#pragma config WRT1 = OFF           // Table Write Protect 04000-07FFF (Disabled)
#pragma config WRT2 = OFF           // Table Write Protect 08000-0BFFF (Disabled)
#pragma config WRT3 = OFF           // Table Write Protect 0C000-0FFFF (Disabled)

// CONFIG6H
#pragma config WRTC = OFF           // Config. Write Protect (Disabled)
#pragma config WRTB = OFF           // Table Write Protect Boot (Disabled)
#pragma config WRTD = OFF           // Data EE Write Protect (Disabled)

// CONFIG7L
#pragma config EBTR0 = OFF          // Table Read Protect 00800-03FFF (Disabled)
#pragma config EBTR1 = OFF          // Table Read Protect 04000-07FFF (Disabled)
#pragma config EBTR2 = OFF          // Table Read Protect 08000-0BFFF (Disabled)
#pragma config EBTR3 = OFF          // Table Read Protect 0C000-0FFFF (Disabled)

// CONFIG7H
#pragma config EBTRB = OFF          // Table Read Protect Boot (Disabled)

void main()
{
		init_Device(); //initialize device
		init_SPI(); //initialize SPO module
		PORTA=0;
		PORTB=0;
		PORTC=0;
		
		RC3=0; //Set clock to low
		RA5=1; //set CS to high
		
		InitializeSPIMode();
		
		for(;;)
		{
		}
}

void init_Device(void) //Initializes the device, setting the oscialltor speed, pin direction, and pull-ups
{
   OSCCON=0b01110111; //Oscillator 8 Mhz
   TRISbits.TRISA5=0; // all Port A output
   TRISB5=1; // make serial port reception pin input
   TRISbits.TRISC4=1; //make SDI reception pin input
   TRISbits.TRISC3=0; // all Port Clock output
   PEIE=1; //enable interrupts for RCIF
   ANSEL=0; //All inputs digital
   ANSELH=0; //All inputs digital
   SBOREN=0; //disable brown out reset
}

void init_SPI(void)
{
   CKP=0; //0=idle clock state is low
   CKE=1; //0=Data transmitted on falling edge of SCK
   SMP=1; //1=SPI mode-Input data sampled at end of data output time
   SSPM3=0; //FOSC/64
   SSPM2=0; //FOSC/64
   SSPM1=0; //FOSC/4 - "1" for /64
   SSPM0=0; //FOSC/64
}

void Delay(long x)
{
   while (x>0){x--;}
}

void WriteSPI(int x) //writes a bit to the memory chip through the SPI module
{
   SSPEN=1; //enable SPI
   SSPBUF=x;
   while (BF==0)
   {;}
   SSPEN=0; //disable SPI
}


void InitializeSPIMode(void)
{
   char x; //used for "while" loops
   unsigned long Address=0;
   unsigned short BytesPerSector=0;
   char ReservedSectors=0;
   char FATCount=0;
   unsigned short SectorsPerFat=0;
   unsigned long FatSpace=0;

   char AddressL=0; //Low byte of address
   char AddressM=0; //Mid byte of address
   char AddressH=0; //High byte of address

   Delay(25000); //Wait one millisecond

	/////////////////////////////////// Slow Oscillator/SPI down for initialization ////////////////////////////////////
	
	OSCCON=0b01000111; //Oscillator 8 Mhz 0b01110111 for 8mhz
	SSPM1=1; //SPI - FOSC/64
	
	////////////////////////////////// Initialize SPI Mode /////////////////////////////////////////////////////////////
	
	WriteSPI(0xFF);WriteSPI(0xFF);WriteSPI(0xFF);WriteSPI(0xFF);WriteSPI(0xFF);WriteSPI(0xFF);WriteSPI(0xFF);WriteSPI(0xFF);WriteSPI(0xFF);WriteSPI(0xFF); //Wake up the card (80 clock pulses)
	
	RA5=0; Delay(5);//Assert CS
	
	WriteSPI(0x40);WriteSPI(0x00);WriteSPI(0x00);WriteSPI(0x00);WriteSPI(0x00);WriteSPI(0x95); //CMD0
	WriteSPI(0);
	WriteSPI(0);
	
	WriteSPI(0x48);WriteSPI(0x00);WriteSPI(0x00);WriteSPI(0x01);WriteSPI(0b10101010);WriteSPI(0x87); //CMD8
	WriteSPI(0);
	WriteSPI(0);

	while(SSPBUF!=0) //ACMD 41 to initialize SD Card - keep doing until you get 0 meaning it's not busy
	{
	   RA5=1; //deassert CS
	   Delay(5);
	   RA5=0; //assert CS
	
	   WriteSPI(0xFF);
	
	   WriteSPI(0b0111011);WriteSPI(0x00);WriteSPI(0x00);WriteSPI(0x00);WriteSPI(0x00);WriteSPI(0xFF); //CMD55
	   WriteSPI(0xFF);
	   WriteSPI(0xFF);
	
	   RA5=1; //deassert CS
	   RA5=0; //assert CS
	
	   WriteSPI(0xFF);
	   WriteSPI(0b01101001);WriteSPI(0b00000000);WriteSPI(0b00000000);WriteSPI(0x00);WriteSPI(0x00);WriteSPI(0xFF); //ACMD41
	   WriteSPI(0xFF);
	   WriteSPI(0xFF);
	}

		////////////////////////////////// Speed Oscillator and SPI back up after initialization /////////////////////////////
	
		OSCCON=0b01110111; //Oscillator 8 Mhz 0b01110111 for 8mhz
		SSPM1=0;
}
