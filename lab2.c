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
#include "hd44780.c"


#define KNOB1 4
#define KNOB2 1

#define DECIMAL 0
#define TWELVEHOUR  1
#define TWENTYFOURHOUR  2

#define Y7 0b01110000
#define DC 0b10001111

#define MODCAP 720

uint8_t debounced_state = 0; // Debounced state of the switches
uint8_t state[MAX_CHECKS]; // Array that maintains bounce status
uint8_t check_index, mode = 0x00; //holds count for display 0; // Pointer into State
static uint8_t enc=0; 
int dim, am=0, aam=0, show_alarm=0, lock = 0, press = 0, button=0, count_add=1;

uint16_t count=0, alarm=0;

void DebounceSwitch();

uint16_t bcd;

//holds data for selecing different digits in the SSD
uint8_t digit_data[5] = {
	0b00000000,//0
	0b00010000,//1
	0b00100000,//:
	0b00110000,//2
	0b01000000 //3
};


//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5] = { 
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111
};

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12] = {
	0b11000000, //0
	0b11111001, //1
	0b10100100, //2
	0b10110000, //3
	0b10011001, //4
	0b10010010, //5
	0b10000010, //6
	0b11111000, //7
	0b10000000, //8
	0b10011000, //9
	0b11111111, //1
	0b10011000 //9
}; 
//initialize SPI mode
void spi_init(void){
	DDRB  |=   0xff;          //Turn on SS, MOSI, SCLK
	SPCR  |=   (1 << SPE) | (1 << MSTR);     //set up SPI mode
	SPSR  |=   (1 << SPI2X);           // double speed operation
}//spi_init

//read values from the encoder and increment count as appropriate
void read_enc(knob){

	static uint8_t enc_state1=0, enc_state2=0, enc_state;
	uint8_t temp = (enc/knob)%4;
	int16_t *target;//isolates 2 digits relevant to encoder


	if(show_alarm) 	target = &alarm;
	else		target = &count;	

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
				if(((*target) - count_add)<0) *target = MODCAP + (*target) - count_add;			
				else *target -= count_add;
			}
			break;
		case 2://two clicks right, two click left
			if(temp == 2) enc_state++;
			if(temp == 1) enc_state--;
			break;
		case 3://three clicks right, one click left
			if(temp == 3){
				enc_state++;
				*target = (*target + count_add);			
			}
			if(temp == 0) enc_state--;
			break;

	}
	if(knob == KNOB1) enc_state1 = enc_state;
	if(knob == KNOB2) enc_state2 = enc_state;

}


//interupt service routine for timercounter0. Used to scan for button presses and data from the ecoder. I have added functions create a delay between presing a button
//and having the controller take action to prevent action from happening when trying to press two buttons at once.  
ISR(TIMER0_OVF_vect){
	static uint16_t count_7ms = 0;
	static uint8_t  button_delay=0, button_release_delay=0;       	//delay counters to create a gap between pressing or releaseing a button and 

	uint16_t am_change=count, aam_change=alarm;

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
	if ((count_7ms % 512)==0) //?? interrupts equals one half second 
		count = (count + 1);//bound the count to 0 - 1023

	if(mode & (1 << 7)) SPDR = mode | (aam << 1);//display current mode on bar graph
	else SPDR = mode | (am << 1);

	while(bit_is_clear(SPSR, SPIF));	//wait until mode is sent to bar graph
	PORTB |=  0x01;        			//strobe output data reg in HC595 - rising edge
	PORTB &=  0xfe;        			//falling edge

	PORTE = 0x00;				//record encoder data
	PORTE = 0x40;				//protect encoder data
	enc = SPDR;				//read encoder data

	read_enc(KNOB1);				//interpreate encoder data and increment/decrement count
	read_enc(KNOB2);				//interpreate encoder data and increment/decrement count
	count = count % MODCAP; 
	alarm = alarm % MODCAP;
	if((am_change <= 100 && count >700) || (am_change > 700 && count <= 100)) am = 1-am;
	if((aam_change <= 100 && alarm >700) || (aam_change > 700 && alarm <= 100)) aam = 1-aam;
}

ISR(TIMER1_COMPA_vect){
	PORTD ^= (1 << 7);
}



//initialize timercounter0
void int0_init(){
	TIMSK |= (1 << TCNT0);			//enable interrupts
	TCCR0 |= (1 << CS02) | (1 << CS00); 	//normal mode, prescale by 128


}
void int1_init() {
	TIMSK |= (1 << OCIE1A);
	TCCR1A = 0x00;
	TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
	TCCR1C = 0x00;
	OCR1A = 0x0031;
	PORTD = 0xff;
	DDRD = 0xff;
} 
/*****************************************************************************
  initialize timer/counter2 to normal mode. Creates the alarm audio signal
 *****************************************************************************/
void int2_init(){
//	TIMSK |= (1 << OCIE2);			//enable interrupts:
	TCCR2 |= (1 << CS20); 
	TCCR2 |= (1 << WGM21) | (1 << WGM20) | (1 << COM21) | (1 << CS20); 	//CTC mode, prescale by 128
	TCCR2 &= ~(1 << FOC2);
	OCR2 = 0xff;
}

/*****************************************************************************
  initialize timer/counter3 to fast pwm mode. Create a duty cycle to control speaker volume
 *****************************************************************************/
void int3_init(){

	TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | (1 << WGM30);//fast pwm, set on match, clear@bottom, 
	TCCR3B |= (1<< WGM32) | (1<<CS30);// | (1 << CS31);//use ICR1 as source for TOP, use clk/1
	TCCR3C= 0x00;                            //no forced compare 
	//	ETIMSK |= (1<<OCIE3A);                         //enable timer 3 interrupt on TOV

	OCR3A = 0x0020;

}
/*****************************************************************************
  initialize the ADC: freerunning mode, clock 1024 prescale
 *****************************************************************************/
void adc_init(){
	PORTF = 0xff; 
	DDRF &= ~(1 << 2);
	ADCSR |= (1 << ADFR);
	ADCSR |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADMUX = 0x02;
	ADMUX |= (1 << REFS0);
	ADMUX |= (1 << ADLAR);
	ADCSR |= (1 << ADEN);
	ADCSR |= (1 << ADIE);
	ADCSR |= (1 << ADSC);

}
ISR(ADC_vect){
	int temp = ADCH;
	if(temp > 350) temp = 350;
	OCR2 = temp;
}

//debouncing switch checks for 12 consecutive signals from same button before returning 1
void DebounceSwitch(){
	uint8_t i,j;
	state[check_index++%MAX_CHECKS]=0xff - PINA;
	j=0xff;
	for(i=0; i<MAX_CHECKS-1;i++)j=j & state[i];
	debounced_state = j;
}
void alarm_on(){
	TIMSK |= (1 << OCIE1A);
}

void alarm_off(){
	TIMSK &= ~(1 << OCIE1A);
}
//returns the display to segmentn_data in decimal
void segsum(uint32_t sum, int c_mode, int am_s) {
	int num_d=0, i;
	uint8_t hour, min;
	int temp, temp2; 
	temp = sum;
	while(sum >= (pow(10, num_d))) {num_d++;}//record number of digits


	switch(c_mode){

		case DECIMAL:
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
			break;
		case TWELVEHOUR:
			hour = (sum/60)%12;
			min = (sum)%60;
			if (hour == 0) hour = 12;


			segment_data[0] = dec_to_7seg[min%10];
			segment_data[1] = dec_to_7seg[min/10];
			segment_data[3] = dec_to_7seg[hour%10];
			if(hour < 10) {segment_data[4] = 0b11111111;}	
			else segment_data[4] = dec_to_7seg[hour/10];
			break;

		case TWENTYFOURHOUR:
			hour = (sum/60)%12;
			min = (sum)%60;

			if(am_s) hour += 12;

			segment_data[0] = dec_to_7seg[min%10];
			segment_data[1] = dec_to_7seg[min/10];
			segment_data[3] = dec_to_7seg[hour%10];
			if(hour < 10) {segment_data[4] = 0b11111111;}	
			else segment_data[4] = dec_to_7seg[hour/10];
			break;


	}
}



uint8_t main()
{
	int i, temp, delay;

	uint8_t aset=(1 << 7), norm=0x00, t_mode=0x00;

	int0_init();
	int1_init();
	int2_init();
	int3_init();
	adc_init();
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
	DDRF = 0xfb;
	DDRE = 0xff;
	PORTE = 0x00;
	
	PORTF &= ~(1 << 0);

	
	while(1)
	{



		if(button>=0){ //if button is pressed toggle mode


			switch(button){
				case 1:
					mode ^= 0x80;
					show_alarm = 1-show_alarm;
					break;
				case 2:
					mode ^= 0x40;
					break;
				case 8 :
					mode ^= 0x01;
					break;
				case 4:
					mode ^= 0x20;
					break;
				case 16:
					alarm_off();
					break;
				case 32:
					alarm_on();
					break;
			}
			switch(mode & (0X7f)){//control count increment value based on mode
				case 0x00:
					count_add = 0;
					break;
				case 0x40:
					count_add = 1;
					break;
				case 0x20:
					count_add = 2;
					break;
				case 0x60:
					count_add = 4;
					break;
			}

			button = -1;	//rell ISR that button has been read and can be changed
		}

		if(show_alarm) 	segsum(alarm, 2, aam);
		else 		segsum(count, 2, am);

		//		sei();         //enable interrupts before entering loop
		for(i=4;i>=0;i--){//bound a counter (0-4) to keep track of digit to display 
			DDRA = 0xff;//make PORTA an output
			PORTA = segment_data[i];//segment_data[i];
			PORTB &= DC;
			PORTB |= digit_data[i];//update digit to display
			_delay_us(300);
			PORTA = 0xff;//isegment_data[i];
			_delay_us(200);
		}
		//		sei();         //enable interrupts before entering loop
	}


	return 0;
}

