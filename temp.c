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
#include"hd44780.h"

#include "ssd_data.c"

#define KNOB1 4
#define KNOB2 1

#define Y7 0b01110000
#define DC 0b10001111

uint8_t debounced_state = 0; // Debounced state of the switches
uint8_t state[MAX_CHECKS]; // Array that maintains bounce status
uint8_t check_index, mode = 0x00; //holds count for display 0; // Pointer into State
static uint8_t enc=0; 
int block = 0, press = 0, button=0, count_add=1;

uint16_t adc_data, audio=0;

uint16_t count=0;

void DebounceSwitch();


uint16_t bcd;

extern uint8_t digit_data[5], segment_data[5], dec_to_7seg[12]; //from ssd_data.c

/*****************************************************************************
initialize SPI mode
*****************************************************************************/
void spi_init(void){
	DDRB  |=   0xff;          //Turn on SS, MOSI, SCLK
	SPCR  |=   (1 << SPE) | (1 << MSTR);     //set up SPI mode
	SPSR  |=   (1 << SPI2X);           // double speed operation
}//spi_init
/*****************************************************************************
** Data is tranlsated from the knob's current position relative to it's last position
*****************************************************************************/
void read_enc(knob){

	static uint8_t enc_state1=0, enc_state2=0, enc_state;
	uint8_t temp = (enc/knob)%4;//isolates 2 digits relevant to encoder

	if(knob == KNOB1) enc_state = enc_state1;
	if(knob == KNOB2) enc_state = enc_state2;

	enc_state=enc_state%4;

	switch(enc_state){
		case 0://neutral position
			if (temp == 1) enc_state++; //indicating we clicked encoder right from neutral position
			if (temp == 2) enc_state --;//indicating we clicked encoder left from neutral position
			break;
		case 1://one click right, three clicks left
			if(temp == 0) enc_state++;
			if(temp == 3){
				enc_state++;
				count = (count - count_add)%1024;			
			}
			break;
		case 2://two clicks right, two click left
			if(temp == 2) enc_state++;
			if(temp == 1) enc_state--;
			break;
		case 3://three clicks right, one click left
			if(temp == 3){
				enc_state++;
				count = (count + count_add)%1024;		
			}
			if(temp == 0) enc_state--;
			break;

	}
	if(knob == KNOB1) enc_state1 = enc_state;
	if(knob == KNOB2) enc_state2 = enc_state;
}


/*****************************************************************************
interupt service routine for timercounter0. Used to scan for button presses and data from the ecoder. I have added functions create a delay between presing a button
and having the controller take action to prevent action from happening when trying to press two buttons at once.  
*****************************************************************************/
ISR(TIMER0_OVF_vect){
	static uint16_t count_7ms = 0;
	static uint8_t  button_delay=0, button_release_delay=0;       	//delay counters to create a gap between pressing or releaseing a button and 
									//and controller taking input from buttons
	count_7ms++;
	PORTA = 0xff;//make PORTA an input port with pullups 
	DDRA = 0x00;

	PORTB |= 0b01110000;//enable tristate buffer for pushbutton switches

	DebounceSwitch();//detect button state

	if(debounced_state && button==-1){				//if button is pressed and was not pressed on last check
		button_delay++;						//wait before taking action. This Allows me to create two button presses
									//that behave differently than one button. No action taken before user can press
									//second button down.

		if ((button_delay > 16) && (button_release_delay > 32)){
			button = debounced_state;
			button_release_delay=0;			//set release to 0 and it will start counting once button is released
		}
	}else{
		button_release_delay++;					//count time since button was released
		button_delay=0;						//button has not been pressed

	}


	//This increments count approximately every 1 second. 
	//	if ((count_7ms % 512)==0) //?? interrupts equals one half second 
	//		count = (count + count_add)%1023;//bound the count to 0 - 1023


	SPDR = mode;//display current mode on bar graph


	while(bit_is_clear(SPSR, SPIF));	//wait until mode is sent to bar graph
	PORTB |=  0x01;        			//strobe output data reg in HC595 - rising edge
	PORTB &=  0xfe;        			//falling edge

	PORTE = 0x00;				//record encoder data
	PORTE = 0x40;				//protect encoder data
	enc = SPDR;				//read encoder data

	read_enc(KNOB1);				//interpreate encoder data and increment/decrement count
	read_enc(KNOB2);				//interpreate encoder data and increment/decrement count
}
/*****************************************************************************
ISR for timer/counter2: set up in fast PWM mode and toggles PE1 to create the alarm signal 
*****************************************************************************/
ISR(TIMER2_COMP_vect){
	PORTE ^= (1 << 0);
}

/*****************************************************************************
ISR for the ADC. Whenever the adc signals a completion it stores the upper 8 bits
*****************************************************************************/
ISR(ADC_vect){


	adc_data = ADCH;
	OCR2  = adc_data;

}

/*****************************************************************************
initialize timer/counter0 to run in normal mode. Scans buttons and enconders and controls the clock display value 
*****************************************************************************/
void int0_init(){
	TIMSK |= (1 << TCNT0);			//enable interrupts
	TCCR0 |= (1 << CS02) | (1 << CS00); 	//normal mode, prescale by 128
}

/*****************************************************************************
initialize timer/counter2 to normal mode. Creates the alarm audio signal
*****************************************************************************/
void int2_init(){
	TIMSK |= (1 << OCIE2);			//enable interrupts:
	TCCR2 |= (1 << WGM21) | (1 << WGM20) | (1 << COM21) | (1 << CS21); 	//CTC mode, prescale by 128
}

/*****************************************************************************
initialize timer/counter3 to fast pwm mode. Create a duty cycle to control speaker volume
*****************************************************************************/
void int3_init(){

	TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | (1 << WGM30);//fast pwm, set on match, clear@bottom, 
	TCCR3B |= (1<< WGM32) | (1<<CS30);// | (1 << CS31);//use ICR1 as source for TOP, use clk/1
	TCCR3C= 0x00;                            //no forced compare 
	ETIMSK = (1<<OCIE3A);                         //enable timer 3 interrupt on TOV
	
	OCR3A = 0x00a0;
	
}
/*****************************************************************************
debouncing switch checks for 12 consecutive signals from same button before returning 1
*****************************************************************************/
void DebounceSwitch(){
	uint8_t i,j;
	state[check_index++%MAX_CHECKS]=0xff - PINA;
	j=0xff;
	for(i=0; i<MAX_CHECKS-1;i++)j=j & state[i];
	debounced_state = j;
}


/*****************************************************************************
returns the display to segmentn_data in decimal
*****************************************************************************/
void segsum(uint16_t sum) {
	int num_d=0, i;
	int temp, temp2;
	temp = sum;
	while(sum >= (pow(10, num_d))) {num_d++;}//record number of digits



	//seg segment_data array correctly
	temp = sum/1000;
	temp2=temp*1000;
	segment_data[4] = dec_to_7seg[temp];
	temp = sum - temp2;
	temp = temp/100;
	temp2+=temp*100;
	segment_data[3] = dec_to_7seg[temp];
	temp = sum - temp2;
	temp = temp/10;
	temp2+=temp*10;
	segment_data[1] = dec_to_7seg[temp];
	temp = sum - temp2;
	segment_data[0] = dec_to_7seg[temp];

	//remove leading 0's
	if(num_d>2) {num_d++;}
	for(i=5;i>num_d;i--) {segment_data[i-1] = 0b11111111;}	

}

/*****************************************************************************
initialize the ADC: freerunning mode, clock 1024 prescale
*****************************************************************************/
void adc_init(){
	PORTF |= (1 << 2);
	DDRF &= ~(1 << 2);
	ADCSR |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	//	ADCSR |= (1 << ADEN);
	ADCSR |= (1 << ADFR);
	ADMUX = 0x62;
	ADCSR |= (1 << ADIE);
	ADCSR |= (1 << ADSC);

}

/*****************************************************************************
main
*****************************************************************************/
uint8_t main()
{
	int i, temp, delay;

	int0_init();
	int2_init();
	int3_init();
	//set port bits 4-7 B as outputs
	spi_init();    //initalize SPI port
	sei();         //enable interrupts before entering loop


	DDRD = 0xff;
	DDRB = 0xff;
	DDRC = 0xff;
	PORTC = 0x00;
	DDRA = 0xff;
	PORTA = 0x00;
	SPDR = 0x01;
	DDRF = 0xff;
	DDRE = 0xff;
	PORTE = 0xff;

	adc_init();
	while(1)
	{


		if(button>=0){ //if button is pressed toggle mode
			switch(button){
				case 1:
					mode ^= 0x80;
					break;
				case 2:
					mode ^= 0x40;
					break;
				case 4:
					mode ^= 0x20;
					break;

			}
			switch(mode){//control count increment value based on mode
				case 0x00:
					count_add = 1;
					break;
				case 0x40:
					count_add = 2;
					break;
				case 0x20:
					count_add = 4;
					break;
				case 0x60:
					count_add = 0;
					break;
			}

			button = -1;	//rell ISR that button has been read and can be changed
		}


		segsum(8888);	//translate count to SSD format

		for(i=4;i>=0;i--){//bound a counter (0-4) to keep track of digit to display 
			DDRA = 0xff;//make PORTA an output
			PORTA = segment_data[i];//segment_data[i];
			PORTB &= DC;
			PORTB |= digit_data[i];//update digit to display
			_delay_us(400);
			PORTA = 0xff;//isegment_data[i];
		}
	}

	return 0;
}
/*
void send_lcd(uint8_t cnd_or_char, uint8_t data, uint16_t wait);
void send_lcd_8bit(uint8_t cnd_or_char, uint8_t data, uint16_t wait);
void set_custom_character(uint8_t data[], uint8_t address);
void set_cursor(uint8_t row, uint8_t col);
void uint2lcd(uint8_t number);
void int2lcd(int8_t number);
void cursor_on(void);
void cursor_off(void);
void shift_right(void);
void shift_left(void);
void cursor_home(void);
void home_line2(void);      
void fill_spaces(void);
void string2lcd(char *lcd_str);
void strobe_lcd(void);
void clear_display(void);
void char2lcd(char a_char);
void lcd_init(void);
void refresh_lcd(char lcd_string_array[]);
void lcd_int32(int32_t l, uint8_t fieldwidth, uint8_t decpos, uint8_t bSigned, uint8_t bZeroFill);
void lcd_int16(int16_t l, uint8_t fieldwidth, uint8_t decpos, uint8_t bZeroFill);
*/



