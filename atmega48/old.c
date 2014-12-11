// thermo3.c
// Zach Murphy
// 11.14.2014

//Demonstrates basic functionality of the LM73 temperature sensor
//Uses the mega128 board and interrupt driven TWI.
//Display is the raw binary output from the LM73.
//PD0 is SCL, PD1 is SDA. 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "lm73_functions_skel.h"
#include "twi_master.h"

#define BAUD 9600


uint8_t i;                     //general purpose index

extern uint8_t *data1;
extern uint8_t *data2;
extern uint8_t num_bytes = 8;
uint8_t lm73_wr_buf[2];
uint8_t lm73_rd_buf[2];

//********************************************************************
//                            spi_init                               
//Initalizes the SPI port on the mega128. Does not do any further    
// external device specific initalizations.                          
//********************************************************************
void spi_init(void){
  DDRB |=  0x07;  //Turn on SS, MOSI, SCLK
  //mstr mode, sck=clk/2, cycle 1/2 phase, low polarity, MSB 1st, 
  //no interrupts, enable SPI, clk low initially, rising edge sample
  SPCR=(1<<SPE) | (1<<MSTR); 
  SPSR=(1<<SPI2X); //SPI at 2x speed (8 MHz)  
 }//spi_init

/***********************************************************************/
/*                                main                                 */
/***********************************************************************/
int main ()
{     
	DDRB = 0xff;
	while(1)
	for(i=0;i<8;i++){
			PORTB ^= 0xff;//(lm73_rd_buf[1]<<i); 
		}

	spi_init();   //initalize SPI 
	init_twi();   //initalize TWI (twi_master.h)  

	//set LM73 mode for reading temperature by loading pointer register
	lm73_wr_buf[0] = LM73_PTR_TEMP;   //load lm73_wr_buf[0] with temperature pointer address
	twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 2);	   //start the TWI write process (twi_master.h)
	_delay_ms(2);      //wait for the xfer to finish

	sei();             //enable interrupts before entering loop

	while(1){          //main while loop
		_delay_ms(100);  //tenth second wait
		twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2);	 //read temperature data from LM73 (2 bytes)  (twi_master.h)
		_delay_ms(2);    //wait for it to finish
		lm73_temp = lm73_rd_buf[0]; //save high temperature byte into lm73_temp
		lm73_temp = (lm73_temp << 8); //shift it into upper byte 
		lm73_temp |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp

		for(i=0;i<8;i++){
			PORTB ^= 0xff;//(lm73_rd_buf[1]<<i); 
		}
	} //while
} //main
