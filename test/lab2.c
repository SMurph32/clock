//Zachary Murphy
//11.5.14


//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#define MAX_CHECKS 12
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void main(){

	PORTE = 0x00;
	DDRE = 0xff;
	
	TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | (1 << WGM30);//fast pwm, set on match, clear@bottom, 
	TCCR3B |= (1<< WGM32) | (1<<CS30);// | (1 << CS31);//use ICR1 as source for TOP, use clk/1
	TCCR1C = 0x00;                            //no forced compare 
	ETIMSK = (1<<OCIE3A);                         //enable timer 3 interrupt on TOV
	
	OCR3A = 0x00fa;
	
	while(1);
}

