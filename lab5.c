// thermo3.c
// Zach Murphy
// 11.14.2014

//Demonstrates basic functionality of the LM73 temperature sensor
//Display is the raw binary output from the LM73.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "lcd_functions.h"
#include "lm73_functions_skel.h"
#include "twi_master.h"
#include "uart_functions.c"
#include "uart_functions.h"

#define RADIO_ADDRESS 0b1100011

char    lcd_string_array[16];  //holds a string to refresh the LCD
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



void radio_init(){
	DDRD |= 0xfc;
	PORD |= (1 << 2);	//TWI
	PORD &= ~(1 << 3);
	_delay_ms(1);		//datasheet requires 300us delay
	PORD |= (1 << 4);	//update by taking RST high
	PORD |= (1 << 5);	//enable voltage regulator




}
/***********************************************************************/
/*                                main                                 */
/***********************************************************************/
int main ()
{     
	uint16_t lm73_temp;  //a place to assemble the temperature from the lm73
	uint8_t cmd;
	int i=0;
	spi_init();   //initalize SPI 

	init_twi();
	radio_init();

	cmd = 0x01;
	twi_start_wr(RADIO_ADDRESS, cmd, 2);

	sei();             //enable interrupts before entering loop
	while(1){          //main while loop

		cmd = 0x01;
		twi_start_wr(RADIO_ADDRESS, cmd, 2);




	}

	return 0;
}
