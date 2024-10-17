#define F_CPU 20000000UL	// Baby Orangutan frequency (20MHz), // F_CPU tells util/delay.h our clock frequency
#define SCL_CLOCK  100000L

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


void PinInit(void);
void TimerOneInit(void);
void GetMagState_0(void);
void GetMagState_1(void);
void GetSlotState_0(void);
void GetSlotState_1(void);
void GetSlotState_2(void);
void GetEnableLineGlobalIn(void);
void GetEnableLineLocalIn(void);
void externalInterruptInit(void);
void setUserLED(unsigned int);
void setCopelyLimitSwitch_0(unsigned int);
void setCopelyLimitSwitch_1(unsigned int);
void setCopelyEnable(unsigned int);
void setDeadmanLED(unsigned int);
void Motor0Init(void);
void Motor1Init(void);
void setMotor_0_Boolean(unsigned int value);
void setMotor_1_Boolean(unsigned int value);

volatile unsigned int magState_0 = 0;
volatile unsigned int magState_1 = 0;
volatile unsigned int slotState_0 = 0;
volatile unsigned int slotState_1 = 0;
volatile unsigned int slotState_2 = 0;
volatile unsigned int enableLineGlobalIn = 0;
volatile unsigned int enableLineLocalIn = 0;
volatile unsigned int deadmanExpiredFlag = 0;

volatile unsigned int userLEDstate = 0;

volatile float main_loop_global_time = 0.0;
volatile float last_interrupt_global_time = 0.0;

#define deadman_time_limit 0.003

ISR(INT0_vect)
{
	//EIMSK &= ~(1 << INT0); //Disable interrupt INT0.
		
	last_interrupt_global_time = main_loop_global_time;
	EIFR |= (1 << INT0); //Manually clear INT0 flag.
}


void delayms( uint16_t millis ) 
{
	while ( millis ) 
	{
		_delay_ms( 1 );
		millis--;
	}
}



int main(void) 
{
	TimerOneInit();
	PinInit();
	Motor0Init();
	Motor1Init();
	externalInterruptInit();
	sei();
//setUserLED(1);
	//setMotor_0_Boolean(1);
	//setMotor_1_Boolean(1);


	while(1)
	{
		if ((TIFR1 & 0b00000010) != 0)
		{
			main_loop_global_time = main_loop_global_time + 0.001; //+ 1 mS

			if(main_loop_global_time - last_interrupt_global_time >= deadman_time_limit)//dead-man timer expired, disable everything, lower enable flag
			{
				deadmanExpiredFlag = 1;
				setDeadmanLED(1);
			}
			else //proceed as normal
			{
				deadmanExpiredFlag = 0;	
				setDeadmanLED(0);
			}

				/////////////////MAIN BODY OF TO-DO CODE IN WHILE(1) LOOP
				GetMagState_0();
				GetMagState_1();
				GetSlotState_0();
				GetSlotState_1();
				GetSlotState_2();
				GetEnableLineGlobalIn();
				GetEnableLineLocalIn();
				/////////////////
			
				/////////////////
				if(magState_0 == 1 || slotState_0 == 1)
				{
					setCopelyLimitSwitch_0(1);
				}
				else
				{
					setCopelyLimitSwitch_0(0);
				}
				/////////////////

				/////////////////
				if(magState_1 == 1 || slotState_1 == 1)
				{
					setCopelyLimitSwitch_1(1);
				}
				else
				{
					setCopelyLimitSwitch_1(0);
				}	
				/////////////////

				

				/////////////////
				if(deadmanExpiredFlag == 1 || enableLineGlobalIn == 0 || enableLineLocalIn == 0)
				{
					setCopelyEnable(0);
				}
				else
				{
					setCopelyEnable(1);
				}
				/////////////////


			TIFR1 |= (1<<OCF1A); //clear interrupt flag
		}
	}

	return 0;
}



void PinInit(void)
{
	
	///////////////////////Input DI pins
	DDRC &= ~(1 << DDC0); // PORTC0 set low to be a digital input for magState_0.
	PORTC |= (1 << PORTC0); //Turn on pull-up resistor.

	DDRC &= ~(1 << DDC1); // PORTC1 set low to be a digital input for magState_1.
	PORTC |= (1 << PORTC1); //Turn on pull-up resistor.

	DDRC &= ~(1 << DDC2); // PORTC2 set low to be a digital input for slotState_0.
	PORTC |= (1 << PORTC2); //Turn on pull-up resistor.

	DDRC &= ~(1 << DDC3); // PORTC3 set low to be a digital input for slotState_1.
	PORTC |= (1 << PORTC3); //Turn on pull-up resistor.

	DDRC &= ~(1 << DDC4); // PORTC4 set low to be a digital input for slotState_2.
	PORTC |= (1 << PORTC4); //Turn on pull-up resistor.

	DDRC &= ~(1 << DDC5); // PORTC5 set low to be a digital input for enableLineGlobalIn.
	//PORTC |= (1 << PORTC5); //Turn on pull-up resistor.

	DDRD &= ~(1 << DDD7); //PORTD7 set hi to be a digital input for enableLineLocalIn..
	//PORTD |= (1 << PORTD7); //Turn on pull-up resistor.

	///////////////////////Output DO pins
	
	DDRD |= (1 << DDD4); //PORTD4 set hi to be a digital output for LED to display state of magState_0.
	DDRD |= (1 << DDD1); //PORTD1 set hi to be a digital output for LED to display state of magState_1. 
	DDRD |= (1 << DDD0); //PORTD0 set hi to be a digital output for LED to display state of slotState_0.
	DDRB |= (1 << DDB5); //PORTB5 set hi to be a digital output for LED to display state of slotState_1.
	DDRB |= (1 << DDB4); //PORTB4 set hi to be a digital output for LED to display state of slotState_2.
	DDRB |= (1 << DDB2); //PORTB2 set hi to be a digital output as for connecting to Copely limit 0 line .
	DDRB |= (1 << DDB1); //PORTB1 set hi to be a digital output as for connecting to Copely limit 1 line.
	DDRB |= (1 << DDB0); //PORTB0 set hi to be a digital output as for connecting to Copely Enable line.
}

void GetMagState_0(void)
{
	if((0b00000001 & PINC) != 0) //PC0
	{
		magState_0 = 0; //Reverses polarity of signal.
		PORTD &= ~(1 << PORTD4); //turn off LED
		//setUserLED(0);
	}
	else
	{
		magState_0 = 1; //Reverses polarity of signal.
		PORTD |= (1 << PORTD4); //turn on LED
		//setUserLED(1);
	}	
}

void GetMagState_1(void)
{
	if((0b00000010 & PINC) != 0) //PC1
	{
		magState_1 = 0; //Reverses polarity of signal.
		PORTD &= ~(1 << PORTD1); //turn off LED
		//setUserLED(0);
	}
	else
	{
		magState_1 = 1; //Reverses polarity of signal.
		PORTD |= (1 << PORTD1); //turn on LED
		//setUserLED(1);
	}	
}

void GetSlotState_0(void)
{
	if((0b00000100 & PINC) != 0) //PC2
	{
		slotState_0 = 0; //Reverses polarity of signal.
		PORTD &= ~(1 << PORTD0); //turn off LED
		//setUserLED(0);
	}
	else
	{
		slotState_0 = 1; //Reverses polarity of signal.
		PORTD |= (1 << PORTD0); //turn on LED
		//setUserLED(1);
	}	
}

void GetSlotState_1(void)
{
	if((0b00001000 & PINC) != 0) //PC3
	{
		slotState_1 = 0; //Reverses polarity of signal.
		PORTB &= ~(1 << PORTB5); //turn off LED		
		//setUserLED(0);
	}
	else
	{
		slotState_1 = 1; //Reverses polarity of signal.
		PORTB |= (1 << PORTB5); //turn on LED
		//setUserLED(1);
	}	
}

void GetSlotState_2(void)
{
	if((0b00010000 & PINC) != 0) //PC4
	{
		slotState_2 = 0; //Reverses polarity of signal.
		PORTB &= ~(1 << PORTB4); //turn off LED		
		//setUserLED(0);
	}
	else
	{
		slotState_2 = 1; //Reverses polarity of signal.
		PORTB |= (1 << PORTB4); //turn on LED
		//setUserLED(1);
	}	
}

void GetEnableLineGlobalIn(void)
{
	if((0b00100000 & PINC) != 0) //PC5
	{
		enableLineGlobalIn = 1; //Does not reverse polarity of signal.
		//setUserLED(0);
	}
	else
	{
		enableLineGlobalIn = 0; //Does not reverse polarity of signal.
		//setUserLED(1);
	}	
}

void GetEnableLineLocalIn(void)
{
	if((0b10000000 & PIND) != 0) //PD7
	{
		enableLineLocalIn = 1; //Does not reverse polarity of signal.
		//setUserLED(0);
	}
	else
	{
		enableLineLocalIn = 0; //Does not reverse polarity of signal.
		//setUserLED(1);
	}	
}

void TimerOneInit(void)
{

	TCCR1A &= ~(1<<WGM10); //Set WGM0 to 0 for CTC (clear timer on compare), mode 4 (TOP = OCR1A). PAGE 137
	TCCR1A &= ~(1<<WGM11); //Set WGM1 to 0 for CTC (clear timer on compare), mode 4 (TOP = OCR1A). PAGE 137

	TCCR1B |= (1 << WGM12); //Set WGM2 to 1 for CTC (clear timer on compare), mode 4 (TOP = OCR1A). PAGE 137

	TCCR1B |= (1<<CS10); //Prescaler set to 1/1024th
	TCCR1B &= ~(1<<CS11); //Prescaler set to 1/1024th
	TCCR1B |= (1<<CS12); //Prescaler set to 1/1024th

	//TIMSK1 |= (1<<OCIE1A); //Enable interrupt when timer 2 matches output compare.
	
	OCR1A = 16; //Outputcompare flags after 2 uS (mathematically should be 32, but minus one for logic)

}

void laserInit(void)
{
/*	laserState = 0;
	DDRD |= (1 << PORTD4);
	setLaser();*/
}

void setCopelyLimitSwitch_0(unsigned int value)
{
	if(value == 0)
	{
		PORTB &= ~(1 << PORTB2);
	}
	else if(value == 1)
	{
		PORTB |= (1 << PORTB2);
	}
}

void setCopelyLimitSwitch_1(unsigned int value)
{
	if(value == 0)
	{
		PORTB &= ~(1 << PORTB1);
	}
	else if(value == 1)
	{
		PORTB |= (1 << PORTB1);
	}
}

void setCopelyEnable(unsigned int value)
{
	if(value == 1)
	{
		PORTB |= (1 << PORTB0);
	}
	else if(value == 0)
	{
		PORTB &= ~(1 << PORTB0);
	}
}

void setUserLED(unsigned int value)
{
	if(value == 1)
	{
		 PORTD |= (1 << PORTD1); //turn on user LED
	}
	else if(value == 0)
	{
		PORTD &= ~(1 << PORTD1); //turn off user LED
	}
}

void externalInterruptInit(void)
{
	//External Pin Interrupts detailed on page 73 of the 328p manual.

	DDRD &= ~(1 << DDD2); //PORTD2 set low to be a digital input for external interrupt.
	EICRA |= (1 << ISC00); //Trigger on toggle.
	EIMSK |= (1 << INT0); //Enable interrupt INT0.
}

void Motor0Init(void)
{
	TCCR0A |= (1 << WGM01);  //Set for fast PWM, mode 3 (TOP = 0xFF).PAGE 161
	TCCR0A |= (1 << WGM00);  //Set for fast PWM, mode (TOP = 0xFF).PAGE 161

	TCCR0A |= (1 << COM0A1);//For pin OC0A to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR0A |= (1 << COM0A0);//For pin OC0A to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR0A |= (1 << COM0B1);//For pin OC0B to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR0A |= (1 << COM0B0);//For pin OC0B to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	
	TCCR0B &= ~(1 << WGM02); //Set WGM2 to 0 for fast PWM, MODE 3 (TOP = 0xFF). PAGE 161

	TCCR0B &= ~(1 << CS00); // 1/8 PAGE 163
	TCCR0B |= (1 << CS01);  // 1/8 PAGE 163
	TCCR0B &= ~(1 << CS02); // 1/8 PAGE 163

	OCR0A = 0;
	OCR0B = 0;

	DDRD |= 1 << DDD5; //Motor 0 control line, Timer0 PWM output B (OC0B)
	DDRD |= 1 << DDD6; //Motor 0 control line, Timer0 PWM output A (OC0A)
}

void Motor1Init(void)
{
	TCCR2A |= (1 << WGM21);  //Set for fast PWM, mode 3 (TOP = 0xFF).PAGE 161
	TCCR2A |= (1 << WGM20);  //Set for fast PWM, mode (TOP = 0xFF).PAGE 161

	TCCR2A |= (1 << COM2A1);//For pin OC1A to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR2A |= (1 << COM2A0);//For pin OC1A to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR2A |= (1 << COM2B1);//For pin OC1B to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR2A |= (1 << COM2B0);//For pin OC1B to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	
	TCCR2B &= ~(1 << WGM22); //Set WGM2 to 0 for fast PWM, MODE 3 (TOP = 0xFF). PAGE 161

	TCCR2B &= ~(1 << CS20); // 1/8 PAGE 163
	TCCR2B |= (1 << CS21);  // 1/8 PAGE 163
	TCCR2B &= ~(1 << CS22); // 1/8 PAGE 163

	OCR2A = 0;
	OCR2B = 0;

	DDRD |= 1 << DDD3; //Motor 2 control line, Timer2 PWM output B (OC2B)	
	DDRB |= 1 << DDB3; //Motor 2 control line, Timer2 PWM output A (OC2A)
}


void setMotor_0_Boolean(unsigned int value)
{
	if(value == 0)
	{
		OCR0A = 0;
		OCR0B = 0;
	}
	else if(value == 1)
	{
		OCR0A = 0;
		OCR0B = 255; //Lights LED with positive terminal on Motor B terminal
	}
}

void setMotor_1_Boolean(unsigned int value)
{
	if(value == 0)
	{
		OCR2A = 0;
		OCR2B = 0;
	}
	else if(value == 1)
	{
		OCR2A = 0;
		OCR2B = 255; //Lights LED with positive terminal on Motor B terminal
	}
}			


void setDeadmanLED(unsigned int value)
{
	if(value == 0)
	{
		setMotor_0_Boolean(0);
	}
	else if(value == 1)
	{
		setMotor_0_Boolean(1);
	}
}
