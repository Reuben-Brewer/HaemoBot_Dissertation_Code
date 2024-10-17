#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

void ADCInit(void);

void TimerZeroInitOC(void);
void InterruptInit(void); 

void SystemInit(void){

	ADCInit();
	TimerZeroInitOC();

	sei();  //Global enable interrupts 
	
}

//Initialize the A to D converter to only run once for every call (setting of the ADSC bit). 
void ADCInit(void){

	ADCSRA |= (1 << ADPS2) | (1 << ADPS1)|(1 << ADPS0); //Sets the AD prescale to 1/128 or ~125kHz
	//ADMUX |= (1 << REFS0);  //Want REFS0 and REFS1 to 0, sets the reference voltage to the AVREF voltage pin on the chip
	ADCSRA |= (1 << ADEN); //ADEN enables ADC

//	Removed code: I don't want to autotrigger, I want to initiate each conversion
//	ADCSRA |= (1 << ADATE); //Sets an auto trigger mode, which is then defined with ADCSRB register ADTS bits

//	Removed code: Don't need to set 0's
//	ADCSRB = (0 << ADTS0) & (0 << ADTS1) & (0 << ADTS2); //Sets to Free Running mode, should not need to actually do

	ADCSRA |= (1 << ADSC);				//Starts the first conversion, since it's single run mode I will need to start each one

//  Removed code: Not using interrupts at the moment
//	ADCSRA |= (1 << ADIE);				//Enable the interrupt at each ADC conversion
}


//Timer used to to run the blinking lights. The timer is initialized to interrupt every
//time the output compare (OC) A and output compare B register values match the timer 0 count
void TimerZeroInitOC(void){

	//Goes to TIMER0_OVF_vect

	//Set prescale to 1/1024
	TCCR0B = (1<<CS02)|(1<<CS00);//|(1<<CS01); 

//	Removed code, I don't want the clock count register to reset after each OC interrupt
//	TCCR0A = (1 << WGM01); //Set the clock register to reset after each OCA is reached

	TIMSK0 |= (1<<OCIE0A);
	TIMSK0 |= (1<<OCIE0B);

	
//	TIMSK0 |= (1<<TOIE0); //Set to interrupt after the overflow (approx .0022s for 7.3728 MHz clock)
	
	OCR0B = 254;
	OCR0A = 255;
	//The overflow flag for this timer is in the TIFR register bit 0

}

