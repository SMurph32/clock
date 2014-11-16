//Zachary Murphy
//10.22.14

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>


#define DC 0b10001111
#define D1 0b00000000
#define D2 0b00010000
#define D3 0b00100000
#define D4 0b00110000
#define D5 0b01000000
#define	DT Ob01110000
//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5] 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12] 

//Debounce_switch taken from Roger Traylor
int8_t debounce_switch(int button) {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PIND, button)) | 0xE000;
  if (state == 0xF000) return 1;
  return 0;
}


//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//
uint8_t chk_buttons(uint8_t button) {
 if((~PINA | (1 << button)) != 0x00){
  if(debounce_switch(button))
   return 1;
 }
 return 0;
}
 

 
//******************************************************************************

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
  //determine how many digits there are 
  //break up decimal sum into 4 digit-segments
  //blank out leading zero digits 
  //now move data to right place for misplaced colon position
}//segment_sum
//***********************************************************************************


//***********************************************************************************
uint8_t main()
{
DDRB = 0x0F;//set port bits 4-7 B as outputs
uint16_t bcd, i, c=0;

while(1){
  _delay_ms(10);//insert loop delay for debounce
  //make PORTA an input port with pullups 
 DDRA = 0xFF;
 PORTA = 0xFF;
 
 PORTB %= DC;
 PORTB |= DT; //enable tristate buffer for pushbutton switches
 c += ~(PINA);//now check each button and increment the count as needed
 c = c%1024; //bound the count to 0 - 1023
 for(i=3;i>=0;i--){
  bcd &= c/;
  bcd << 4;
  if((c&(1<<i) //break up the disp_value to 4, BCD digits in the array: call (segsum)
  //bound a counter (0-4) to keep track of digit to display 
  //make PORTA an output
  //send 7 segment code to LED segments
  //send PORTB the digit to display
  //update digit to display
  }//while
}//main
