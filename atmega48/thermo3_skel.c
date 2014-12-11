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


int main ()
{     
	int i;
	DDRB = 0xff;
	while(1)
		for(i=0;i<8;i++){
			PORTB ^= 0xff;//(lm73_rd_buf[1]<<i); 
		}
} //main
