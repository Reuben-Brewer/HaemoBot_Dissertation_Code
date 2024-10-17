#define F_CPU 20000000UL	// Baby Orangutan frequency (20MHz), // F_CPU tells util/delay.h our clock frequency
#define SCL_CLOCK  100000L
#define TxFlag 1
#define hysteresis_threshold_pound_define 1
//////////////////////////////////////////////

//////////////////////////////////////////////
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//////////////////////////////////////////////

//////////////////////////////////////////////
void PinInit(void);
void TimerOneAsPWMInit(void);
void SerialInit(void);
unsigned int CheckAndWriteNewBooleanSerialValue(unsigned int oldVal, unsigned int newVal);
void setLaser808NMpinState(unsigned int value);

void setDebugLED(unsigned int LED_num, unsigned int value);
void toggleDebugLED(unsigned int LED_num);

void ADCInit(void);
void ADCSetChannel(unsigned char channel);
void checkSwitches(void);
//////////////////////////////////////////////

//////////////////////////////////////////////
volatile unsigned int TxCounter = 0;
volatile unsigned int TxMessageNum = 0;

volatile unsigned long int main_loop_counter = 0;
volatile float debug_to_computer;

volatile unsigned int LEDstate[6];
volatile unsigned int Laser808NMblockSignal = 0;

volatile unsigned int Laser808NMbrightnessLevelAnalog = 1;
volatile unsigned int last_Laser808NMbrightnessLevelAnalog = 2048;
volatile float DC_percentage = 0.001;
volatile unsigned int Laser808NM_register_value_OCR1A = 0;
volatile unsigned int Laser808NM_register_value_OCR1B = 0;
volatile unsigned char Laser808NM_frequency = 0;
volatile unsigned int hysteresis_threshold = 0;

volatile unsigned int ADCcounter;
volatile unsigned char ADCloByte;
volatile unsigned char ADChiByte;
volatile unsigned char has_Laser808NMbrightnessLevelAnalog_been_read = 0;

volatile unsigned char tempTx;

volatile float TimerOnePrescalerN;
volatile unsigned int OCR1A_max;
volatile unsigned int timer1_TOP;

volatile unsigned char touch_switch_1 = 0;
volatile unsigned char touch_switch_2 = 0;
volatile unsigned char touch_switch_3 = 0; 
volatile unsigned char reed_switch_overide_RSO = 0;
//////////////////////////////////////////////

//////////////////////////////////////////////
typedef union //c-syntax
{
	float float_num;
	char char_num[4];
} my_union; //c-syntax
//////////////////////////////////////////////

//////////////////////////////////////////////
inline void ADCReadAndRespond(unsigned char channel)
{
	ADCloByte = ADCL;
	ADChiByte	= ADCH;
	Laser808NMbrightnessLevelAnalog = (ADCloByte |(ADChiByte<<8));

	has_Laser808NMbrightnessLevelAnalog_been_read = 1;

	ADCSRA |= (1 << ADIF); //Clear the A to D conversion complete flag (done by setting it to 1)
	ADCSRA |= (1 << ADSC); //Initialize the next A to D conversion
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline void setLaser808NMpinState(unsigned int value)
{
	if(value == 0)
	{
		Laser808NMblockSignal = 1;
	}
	else if(value == 1)
	{
		Laser808NMblockSignal = 0;
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline  void CalcAndSetTimer1registerLaser808NM(void)
{
	//DC_percentage = (float) 100.0*Laser808NMbrightnessLevelAnalog/1024.0;

	DC_percentage = 100.0 - (float) 100.0*Laser808NMbrightnessLevelAnalog/1024.0;

	if(DC_percentage < (float) 1.0*100.0/1024.0)
	{
		DC_percentage = (float) 1.0*100.0/1024.0;
	}

	if(Laser808NMblockSignal == 1)
	{
		DC_percentage = 0;
	}

	OCR1A = (float) (DC_percentage/100.0)*OCR1A_max;

	last_Laser808NMbrightnessLevelAnalog = Laser808NMbrightnessLevelAnalog;	
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline void send_serial_byte(unsigned char byte)
{
	while ( !( UCSR0A & (1<<UDRE0)) )
	{
	}
	UDR0 = byte;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline void send_serial_int(int num)
{
	unsigned char lo_byte, hi_byte;

	lo_byte = num;
	hi_byte = num>>8;

	send_serial_byte(lo_byte);
	send_serial_byte(hi_byte);
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline void send_serial_float(float num)
{
	my_union Tx_float_union;
	Tx_float_union.float_num = num;

	send_serial_byte(Tx_float_union.char_num[0]);
	send_serial_byte(Tx_float_union.char_num[1]);
	send_serial_byte(Tx_float_union.char_num[2]);
	send_serial_byte(Tx_float_union.char_num[3]);
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void delayms( uint16_t millis ) 
{
	while ( millis ) 
	{
		_delay_ms( 1 );
		millis--;
	}
}
//////////////////////////////////////////////


//////////////////////////////////////////////
int main(void) 
{	
	PinInit();
	ADCInit();
	TimerOneAsPWMInit();
	SerialInit();
	sei();
		setLaser808NMpinState(0);
		Laser808NMbrightnessLevelAnalog = 0;
		CalcAndSetTimer1registerLaser808NM();
		last_Laser808NMbrightnessLevelAnalog = 2048;
	
	while(1)
	{	

		////////////////////////////////////////////// Read touch sensors.
		checkSwitches();

		//If the reed_switch_overide_RSO is on
		if(reed_switch_overide_RSO == 1)
		{
			setLaser808NMpinState(1); 
		}
		//If the reed switch override is off and all of the touch switches are being pressed
		else if((touch_switch_1 == 1) && (touch_switch_2 == 1) && (touch_switch_3 == 1)) 
		{
			setLaser808NMpinState(1); //turn off the 808nm laser.
		}
		//If the reed_switch_overide_RSO is off and some of the touch sensors aren't being pressed
		else
		{
			setLaser808NMpinState(0); 
		}
		//////////////////////////////////////////////

		////////////////////////////////////////////// Read analog in
		if ((ADCSRA & 0b00010000) != 0)
		{
			ADCReadAndRespond(0);	
		}
		//////////////////////////////////////////////

		setDebugLED(4, Laser808NMblockSignal);

		//////////////////////////////////////////////Set 808nm laser output power
		hysteresis_threshold = hysteresis_threshold_pound_define;
		if(has_Laser808NMbrightnessLevelAnalog_been_read == 1) //Make sure we've got a proper ADC reading before we start setting the 808NM laser.
		{
			if((Laser808NMbrightnessLevelAnalog >= (last_Laser808NMbrightnessLevelAnalog + hysteresis_threshold)) || (Laser808NMbrightnessLevelAnalog <= (last_Laser808NMbrightnessLevelAnalog - hysteresis_threshold)))
			{
				//toggleDebugLED(4);
				CalcAndSetTimer1registerLaser808NM();
			}
		}
		//////////////////////////////////////////////		

		//////////////////////////////////////////////Serial Tx
		TxCounter = TxCounter + 1;
		if(TxCounter == 5000) 
		{			
			TxMessageNum = TxMessageNum + 1;
			if(TxFlag == 1)
			{
				//toggleDebugLED(4);

				tempTx = DC_percentage; 	
				//tempTx = Laser808NMbrightnessLevelAnalog; 
				//tempTx = capacitive_switch_3;
				//send_serial_byte(tempTx); 

				send_serial_byte(DC_percentage); 
				send_serial_byte(Laser808NMbrightnessLevelAnalog); 
					
			}
			TxCounter = 0;
		}
		//////////////////////////////////////////////
				
		main_loop_counter = main_loop_counter + 1;
	}

	return 0;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void ADCInit(void)
{
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1)|(1 << ADPS0); //Sets the AD prescale to 1/128 or 156.25kHz (page 265)
	ADCSRA |= (1 << ADEN); //Enables ADC.

	ADCcounter = 0;
    ADCSetChannel(ADCcounter); //must jump start the conversion so that we have a flag to look for in the main() loop

	DDRC &= ~(1 << DDC0); // PORTC0 set low to be an analog input.
	//DDRC &= ~(1 << DDC1); // PORTC1 set low to be an analog input.

	ADCSRA |= (1 << ADIF); //Clear the A to D conversion complete flag (done by setting it to 1)
	ADCSRA |= (1 << ADSC); //Initialize the next A to D conversion
}
//////////////////////////////////////////////

//////////////////////////////////////////////
//This function sets the proper ADC channel to pay attention to then starts the conversion
void ADCSetChannel(unsigned char channel)
{
	if(channel == 0)
	{
		ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0)); //Clear bits 0:4 to select ADC0. Page 291
	}
	else if(channel == 1)
	{
		ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0)); //Clear bits 0:4, set bit 1 high to select ADC1. Page 291;
		ADMUX |= (1 << MUX0); //Page 291

	}
	else if(channel == 2)
	{
		ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0)); //Clear bits 0:4, set bit 1 high to select ADC2. Page 291;
		ADMUX |= (1 << MUX1); //Page 291
	}
	else if(channel == 3)
	{
		ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0)); //Clear bits 0:4, set bit 1 high to select ADC3. Page 291;
		ADMUX |= (1 << MUX1) | (1 << MUX0); //Page 291
	}
	else if(channel == 4)
	{
		ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0)); //Clear bits 0:4, set bit 1 high to select ADC6. Page 291;
		ADMUX |= (1 << MUX2) | (1 << MUX1); //Page 291
	}
}
//////////////////////////////////////////////


//////////////////////////////////////////////
void SerialInit(void)
{
	UCSR0A = (1 << U2X0); //double the transmission speed
	UCSR0B = (1 << RXCIE0) | (1 << TXEN0) | (1 << RXEN0); //enable RX Complete Interrupt, enable Tx, enable Rx
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8n1
	UBRR0H = 0;
	UBRR0L = 4; // FOR U2X0 = 1 (doubling Tx speed), UBRR0L = 4 for 0.5Mbs, UBRR0L = 21 for 115k2 page 180, page 199 for baud rate example table. MATLAB SCRIPT FOR CALCULATING BAUD RATE

	DDRD &= ~(1 << DDD0); // PORTD0 is set as input for UART RX.
	PORTD |= (1 << DDD0); //Turn on pull-up resistor.
	DDRD |= (1 << DDD1); // PORTD1 is set as output for UART TX.
}
//////////////////////////////////////////////

//////////////////////////////////////////////
unsigned int CheckAndWriteNewBooleanSerialValue(unsigned int oldVal, unsigned int newVal)
{
	if(newVal != 0 && newVal != 1)
	{
		return oldVal;
	}
	else
	{
		return newVal;
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void PinInit(void)
{
	///////////////////////Input pins
	//DDRC &= ~(1 << DDC0); // PORTC0 set low to be an analog input.
	//DDRC &= ~(1 << DDC1); // PORTC1 set low to be an analog input.

	DDRC &= ~(1 << DDC2); // PORTC2 set low to be a digital input for capacitive_switch_1.
	PORTC |= (1 << PORTC2); //Turn on pull-up resistor.

	DDRC &= ~(1 << DDC3); // PORTC3 set low to be a digital input for capacitive_switch_2.
	PORTC |= (1 << PORTC3); //Turn on pull-up resistor.

	DDRC &= ~(1 << DDC4); // PORTC4 set low to be a digital input for capacitive_switch_3.
	PORTC |= (1 << PORTC4); //Turn on pull-up resistor.

	DDRC &= ~(1 << DDC5); // PORTC5 set low to be a digital input for reed_switch_overide_RSO.
	PORTC |= (1 << PORTC5); //Turn on pull-up resistor.
	///////////////////////

	///////////////////////Output pins	
	//DDRB |= 1 << DDB1; //Timer1 PWM output B (OC1B)	
	//DDRB |= 1 << DDB2; //Timer1 PWM output A (OC1A)


	DDRB |= (1 << DDB0); // PORTBO set HI to be a digital output for LED_1.
	DDRB |= (1 << DDB4); // PORTB4 set HI to be a digital output for LED_2.
	DDRB |= (1 << DDB5); // PORTB5 set HI to be a digital output for LED_3.
	DDRD |= (1 << DDD2); // PORTD2 set HI to be a digital output for LED_4.
	DDRD |= (1 << DDD4); // PORTD4 set HI to be a digital output for LED_5.
	DDRD |= (1 << DDD7); // PORTD7 set HI to be a digital output for LED_6.
	///////////////////////
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void setDebugLED(unsigned int LED_num, unsigned int value)
{
	LEDstate[LED_num] = value;	

	////////////////////////
	if(LED_num == 0)
	{
		if(value == 1)
		{
			 PORTB |= (1 << PORTB0); //turn on LED_1	 
		}
		else if(value == 0)
		{
			PORTB &= ~(1 << PORTB0); //turn off user LED_1
		}
	}
	////////////////////////
	////////////////////////
	else if(LED_num == 1)
	{
		if(value == 1)
		{
			 PORTB |= (1 << PORTB4); //turn on LED_2
		}
		else if(value == 0)
		{
			PORTB &= ~(1 << PORTB4); //turn off user LED_2
		}
	}
	////////////////////////
	////////////////////////
	else if(LED_num == 2)
	{
		if(value == 1)
		{
			 PORTB |= (1 << PORTB5); //turn on LED_3
		}
		else if(value == 0)
		{
			PORTB &= ~(1 << PORTB5); //turn off user LED_3
		}
	}
	////////////////////////
	////////////////////////
	else if(LED_num == 3)
	{
		if(value == 1)
		{
			 PORTD |= (1 << PORTD2); //turn on LED_4
		}
		else if(value == 0)
		{
			PORTD &= ~(1 << PORTD2); //turn off user LED_4
		}
	}
	////////////////////////
	////////////////////////
	else if(LED_num == 4)
	{
		if(value == 1)
		{
			 PORTD |= (1 << PORTD4); //turn on LED_5
		}
		else if(value == 0)
		{
			PORTD &= ~(1 << PORTD4); //turn off user LED_5
		}
	}
	////////////////////////
	////////////////////////
	else if(LED_num == 5)
	{
		if(value == 1)
		{
			 PORTD |= (1 << PORTD7); //turn on LED_6
		}
		else if(value == 0)
		{
			PORTD &= ~(1 << PORTD7); //turn off user LED_6
		}
	}
	////////////////////////
}
//////////////////////////////////////////////


//////////////////////////////////////////////
void toggleDebugLED(unsigned int LED_num)
{
	if(LEDstate[LED_num] == 0)
	{
		setDebugLED(LED_num, 1);
	}
	else if(LEDstate[LED_num] == 1)
	{
		setDebugLED(LED_num, 0);
	}
}
//////////////////////////////////////////////


//////////////////////////////////////////////
void TimerOneAsPWMInit(void)
{
	TCCR1A &= ~(1 << WGM10);  //Set for fast PWM, mode 14 (TOP = ICR1).PAGE 137
	TCCR1A |= (1 << WGM11);  //Set for fast PWM, mode 14 (TOP = ICR1).PAGE 137
	TCCR1B |= (1 << WGM12);  //Set for fast PWM, mode 14 (TOP = ICR1).PAGE 137
	TCCR1B |= (1 << WGM13);  //Set for fast PWM, mode 14 (TOP = ICR1).PAGE 137

	TCCR1A |= (1 << COM1A1);//For pin OC1A to be clear on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR1A &= ~(1 << COM1A0);//For pin OC1A to be clear on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR1A |= (1 << COM1B1);//For pin OC1B to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR1A &= ~(1 << COM1B0);//For pin OC1B to be set on compare match, FAST-PWM, inverting mode.PAGE 160

	TimerOnePrescalerN = 8;
	TCCR1B &= ~(1 << CS10);//  0	Prescaler set to 1/8th page 138 
	TCCR1B |= (1 << CS11); //  1	Prescaler set to 1/8th page 138 
	TCCR1B &= ~(1 << CS12);//  0	Prescaler set to 1/8th page 138 

	Laser808NM_frequency = 200.0; 
	timer1_TOP = 20000000.0/(Laser808NM_frequency*TimerOnePrescalerN)-1;
	OCR1A_max = timer1_TOP;

	ICR1 = timer1_TOP;
	
	DDRB |= 1 << DDB1; //Timer1 PWM output B (OC1B)	
	DDRB |= 1 << DDB2; //Timer1 PWM output A (OC1A)
}
//////////////////////////////////////////////

void checkSwitches(void)
{
	//////////////////////////////
	if((0b00000100 & PINC) != 0) //PC2
	{
		touch_switch_1 = 0;
		setDebugLED(0, 0); 
	}
	else
	{
		touch_switch_1 = 1; //Switch OFF corresponds to GND voltage.
		setDebugLED(0, 1); 
	}
	//////////////////////////////

	//////////////////////////////
	if((0b00001000 & PINC) != 0) //PC3
	{
		touch_switch_2 = 0; 
		setDebugLED(1, 0); 
	}
	else
	{
		touch_switch_2 = 1; //Switch OFF corresponds to GND voltage.
		setDebugLED(1, 1); 
	}
	//////////////////////////////

	//////////////////////////////
	if((0b00010000 & PINC) != 0) //PC4
	{
		touch_switch_3 = 0; 
		setDebugLED(2, 0); 
	}
	else
	{
		touch_switch_3 = 1; //Switch ON corresponds to GND voltage.
		setDebugLED(2, 1);  
	}
	//////////////////////////////

	//////////////////////////////
	if((0b00100000 & PINC) != 0) //PC5
	{
		reed_switch_overide_RSO = 0; 
		setDebugLED(3, 0); 
	}
	else
	{
		reed_switch_overide_RSO = 1; //Switch OFF corresponds to GND voltage.
		setDebugLED(3, 1); 
	}
	//////////////////////////////

}

