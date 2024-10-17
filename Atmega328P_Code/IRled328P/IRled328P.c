#define F_CPU 20000000UL	// Baby Orangutan frequency (20MHz), // F_CPU tells util/delay.h our clock frequency
#define SCL_CLOCK  100000L

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void Motor0Init(void);
void Motor1Init(void);
void ADCInit(void);
void ADCSetChannel(unsigned char channel);
void TouchSensorInit(void);
void TimerOneInit(void);
void TimerOneAsPWMInit(void);
void Timer0asLoopTimerInit(void);
void SerialInit(void);
void GetSerialORanalogMode(void);
unsigned int CheckAndWriteNewBooleanSerialValue(unsigned int oldVal, unsigned int newVal);

volatile unsigned int ADCcounter;
volatile unsigned char ADCloByte;
volatile unsigned char ADChiByte;
volatile unsigned int LEDlevelAnalog = 0;
volatile unsigned int touchState = 0;
volatile unsigned int TxCounter = 0;
volatile float main_loop_global_time = 0.0;
volatile float OCR1A_max = 20000.0; //max value of 200 will give us a pulse of 10uS, 250 for 100hz

volatile unsigned int Rx_mutex_flag;
volatile float last_Rx_time = 0.0;
volatile float debug_to_computer;
volatile unsigned int counterRx = 0;
volatile unsigned int message_counter = 0;
volatile unsigned int message_length = 0;
volatile unsigned int RxMessage[11];
volatile unsigned int LEDlevelRx[9] = {0,0,0,0,0,0,0,0,0};
volatile unsigned int LEDlevel = 0;
volatile unsigned int serialORanalogMode = 0;
volatile unsigned int received_checksum, calculated_checksum;
volatile int message_being_processed = 0;
volatile unsigned int motor_goal_pos[2] = {0, 0};
volatile unsigned int motor_goal_vel[2] = {0, 0};
volatile unsigned int motor0_limit_switch[2] = {0, 0};
volatile unsigned int motor1_limit_switch[2] = {0, 0};
volatile unsigned int cathHomeSwitch = 0;

#define thisLEDnumber 0
#define dead_man_enabled 0
#define dead_man_time 0.05
#define TxFlag 1
#define numADCchannels 1



typedef union //c-syntax
{
	float float_num;
	char char_num[4];
} my_union; //c-syntax

inline signed int check_deadman_expired(void)
{
	if(dead_man_enabled == 1)
	{
		if(main_loop_global_time - last_Rx_time < dead_man_time)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return -1;
	}
}


inline void ADCReadAndRespond(unsigned char channel)
{

	ADCloByte = ADCL;
	ADChiByte	= ADCH;
	LEDlevelAnalog = (ADCloByte |(ADChiByte<<8));
}

inline void send_serial_byte(unsigned char byte)
{
	while ( !( UCSR0A & (1<<UDRE0)) )
	{
	}
	UDR0 = byte;
}


inline void send_serial_int(int num)
{
	unsigned char lo_byte, hi_byte;

	lo_byte = num;
	hi_byte = num>>8;

	send_serial_byte(lo_byte);
	send_serial_byte(hi_byte);
}

inline void send_serial_float(float num)
{
	my_union Tx_float_union;
	Tx_float_union.float_num = num;

	send_serial_byte(Tx_float_union.char_num[0]);
	send_serial_byte(Tx_float_union.char_num[1]);
	send_serial_byte(Tx_float_union.char_num[2]);
	send_serial_byte(Tx_float_union.char_num[3]);
}



void delayms( uint16_t millis ) 
{
	while ( millis ) 
	{
		_delay_ms( 1 );
		millis--;
	}
}


ISR(USART_RX_vect)
{
	int headerCounter;

	if(message_being_processed == 0)
	{
		//Shift old data down by one slot.
		
		for(headerCounter = 0; headerCounter <= 3; headerCounter++)
		{
			RxMessage[headerCounter] = RxMessage[headerCounter+1];
		}

		RxMessage[4] = UDR0; //Grab new serial data.

		//If we've received a proper header, then set the flag so that we start processing new message.
		if(RxMessage[0] == 0xFF  && RxMessage[1] == 0x00  && RxMessage[2] == 0xFF  && RxMessage[3] == 0x00)
		{
			message_being_processed = 1;
			message_counter = 5;

			if(RxMessage[4] == 1)
			{
				message_length = 14;  //Message length counting message bytes from 0.
			}
			else if(RxMessage[4] == 2)
			{
				message_length = 10; //Message length counting message bytes from 0.
			}
		}
	}

	else if(message_being_processed == 1)
	{
		if(message_counter <= message_length) //Keep grabbing new serial data	
		{
			RxMessage[message_counter] = UDR0; //Grab new serial data.
			message_counter = message_counter + 1;
		}

		if(message_counter > message_length) //Stop grabbing serial data and parse message
		{
			
			message_being_processed = 0;
			//UCSR0B &= (0 << RXEN0); //disable USART0 Rx while we process message.
			
			int ledNum;
			if(RxMessage[4] == 1) // Motor position message
			{		
				//FF--00--FF--00--MESSAGE_NUM(1)--LED0--LED1--LED2--LED3--LED4--LED5--LED6--LED7--LED8---CHECKSUM
				//0   1   2   3   4               5     6     7     8     9     10    11    12    13     14
				
			
				Rx_mutex_flag = 1;

					for(ledNum = 0; ledNum < 9; ledNum++)
					{
						LEDlevelRx[ledNum] = RxMessage[5+ledNum];
					}

					received_checksum = RxMessage[14];

				Rx_mutex_flag = 0;
			}
			else if(RxMessage[4] == 2) //unused message as of yet
			{
				
				
				//FF--00--FF--00--MESSAGE_NUM(2)--CHECKSUM
				//0   1   2   3   4               5 
				//received_checksum = RxMessage[8];	

			}


			//Send back RxMessage to computer for debugging.
		/*	for(int i = 0; i <= message_length; i++)
			{
				send_serial_byte(RxMessage[i]);
			}*/

			last_Rx_time = main_loop_global_time;
			counterRx++;

		}	
	}
}

ISR(ADC_vect)
{
	ADCSRA &= ~(1 << ADIE); //Disables ADC interrupts.
	ADCReadAndRespond(ADCcounter);	
	ADCSetChannel(ADCcounter);

	ADCSRA |= (1 << ADIF); //Clear the A to D conversion complete flag (done by setting it to 1)
	ADCSRA |= (1 << ADIE); //Enables ADC interrupts.
	ADCSRA |= (1 << ADSC); //Initialize the next A to D conversion
}

int main(void) 
{
	


	
	Timer0asLoopTimerInit();
	ADCInit();
	TimerOneAsPWMInit();
	SerialInit();
	sei();



	while(1)
	{

		

	
		if ((TIFR0 & 0b00000010) != 0)
		{
			main_loop_global_time = main_loop_global_time + 0.001; //+ 1 mS

				if(Rx_mutex_flag == 0)
				{

					GetSerialORanalogMode();

					if(check_deadman_expired() != 1)
					{
						if(serialORanalogMode == 0)
						{
							LEDlevel =  (OCR1A_max/200)*LEDlevelRx[thisLEDnumber];
						}
						else
						{
							LEDlevel =  (OCR1A_max/1024)*LEDlevelAnalog;
						}
					}
					else
					{
						LEDlevel = 0;
					}

					if(LEDlevel > OCR1A_max)
					{
						LEDlevel = OCR1A_max;
					}
					if(LEDlevel < 0)
					{
						LEDlevel = 0;
					}

					OCR1A = LEDlevel;

				}

			



			TxCounter = TxCounter + 1;
			if(TxCounter == 1000) //every 10ms
			{

				if(TxFlag == 1)
				{
					send_serial_byte(0xFF); //byte 0
					send_serial_byte(0x00); //byte 1
					send_serial_byte(0xFF); //byte 2
					send_serial_byte(0x00); //byte 3
					send_serial_byte(0x01); //byte 4 Message Type 1
					send_serial_float(main_loop_global_time); //byte 5:8
					send_serial_byte((unsigned char) LEDlevel); //byte 9

					float temp = main_loop_global_time - last_Rx_time;
					debug_to_computer = temp;
					send_serial_float(debug_to_computer); //bytes 10:13 debug message

					send_serial_byte(77); //byte 14, checksum
					send_serial_byte((unsigned char) LEDlevelAnalog);
										
				}

				TxCounter = 0;
			}


			
			TIFR0 |= (1<<OCF1A); //clear interrupt flag
		}

	
	}

	return 0;
}


void Motor0Init(void)
{
	TCCR0A |= (1 << WGM01);  //Set for fast PWM, mode 3 (TOP = 0xFF).PAGE 161
	TCCR0A |= (1 << WGM00);  //Set for fast PWM, mode (TOP = 0xFF).PAGE 161

	TCCR0A |= (1 << COM0A1);//For pin OC0A to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR0A |= (1 << COM0A0);//For pin OC0A to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR0A |= (1 << COM0B1);//For pin OC0B to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR0A |= (1 << COM0B0);//For pin OC0B to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	
	TCCR0B &= (1 << WGM02); //Set WGM2 to 0 for fast PWM, MODE 3 (TOP = 0xFF). PAGE 161

	TCCR0B &= ~(1 << CS00); // 1/8 PAGE 163
	TCCR0B |= (1 << CS01);  // 1/8 PAGE 163
	TCCR0B &= ~(1 << CS02); // 1/8 PAGE 163

	OCR0A = 0;
	OCR0B = 0;

	DDRD |= 1 << DDD5; //Motor 0 control line, Timer0 PWM output B (OC0B)
	DDRD |= 1 << DDD6; //Motor 0 control line, Timer0 PWM output A (OC0A)
}

void Timer0asLoopTimerInit(void)
{
	TCCR0A &= ~(1<<WGM00); //Set WGM0 to 0 for CTC (clear timer on compare), mode 4 (TOP = OCR1A). PAGE 137
	TCCR0A &= ~(1<<WGM01); //Set WGM1 to 0 for CTC (clear timer on compare), mode 4 (TOP = OCR1A). PAGE 137

	TCCR0B |= (1 << WGM02); //Set WGM2 to 1 for CTC (clear timer on compare), mode 4 (TOP = OCR1A). PAGE 137

	TCCR0B |= (1<<CS00); //Prescaler set to 1/1
	TCCR0B &= ~(1<<CS01); //Prescaler set to 1/1
	TCCR0B &= ~(1<<CS02); //Prescaler set to 1/1
	
	OCR0A = 20000-1; //Outputcompare flags for 4kHz (minus one for logic)
}

void ADCInit(void)
{

	ADCSRA |= (1 << ADPS2) | (1 << ADPS1)|(1 << ADPS0); //Sets the AD prescale to 1/128 or 156.25kHz
	ADCSRA |= (1 << ADEN); //Enables ADC.

    ADCSetChannel(ADCcounter); //must jump start the conversion so that we have a flag to look for in the main() loop

	DDRC &= ~(1 << DDC0); // PORTC0 set low to be an analog input.

	ADCSRA |= (1 << ADIF); //Clear the A to D conversion complete flag (done by setting it to 1)
	ADCSRA |= (1 << ADIE); //Enables ADC interrupts.
	ADCSRA |= (1 << ADSC); //Initialize the next A to D conversion
}



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

void TouchSensorInit(void)
{
	DDRD &= ~(1 << DDD2); // PORTD2 set low to be a digital input, no pull-up resistor.
	PORTD |= (1 << PORTD2);

	DDRD |= (1 << DDD7); // PORTD7 set high to be a digital output.
}

void GetSerialORanalogMode(void)
{
	if((0b00000100 & PIND) != 0) //D2
	{
		serialORanalogMode = 1;
	}
	else
	{
		serialORanalogMode = 0;
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


	TCCR1B |= (1 << CS10); // 1/1 PAGE 138
	TCCR1B &= ~(1 << CS11); // 1/1 PAGE 138
	TCCR1B &= ~(1 << CS12); // 1/1 PAGE 138 

/*	TCCR1B &= ~(1 << CS10); // 1/8 PAGE 138
	TCCR1B |= (1 << CS11); // 1/8 PAGE 138
	TCCR1B &= ~(1 << CS12); // 1/8 PAGE 138
	
	ICR1H = (25000>>8); for 100hz and 100us
	ICR1L = 25000;
	*/

	ICR1H = (20000>>8); //20000m 200 for 1khz and 10us
	ICR1L = 20000;
	
	//OCR1A = 200;

	DDRB |= 1 << DDB1; //Motor 2 control line, Timer2 PWM output B (OC2B)	
	DDRB |= 1 << DDB2; //Motor 2 control line, Timer2 PWM output A (OC2A)

}

void SerialInit(void)
{
	UCSR0A = (1 << U2X0); //double the transmission speed
	UCSR0B = (1 << RXCIE0) | (1 << TXEN0) | (1 << RXEN0); //enable RX Complete Interrupt, enable Tx, enable Rx
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8n1
	UBRR0H = 0;
	UBRR0L = 21; // 115k2 UBBR=(f_osc/(16*BAUD))-1, page 159

	DDRD &= ~(1 << DDD0); // PORTD0 is set as input for UART RX.
	PORTD |= (1 << DDD0); //Turn on pull-up resistor.
	DDRD |= (1 << DDD1); // PORTD1 is set as output for UART TX.
}


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


/*  PHASE AND FREQUENCY CORRECT PWM WORKS
void TimerOneAsPWMInit(void)
{
	TCCR1A &= ~(1 << WGM10);  //Set for fast PWM, mode 14 (TOP = ICR1).PAGE 137
	TCCR1A &= ~(1 << WGM11);  //Set for fast PWM, mode 14 (TOP = ICR1).PAGE 137
	TCCR1B &= ~(1 << WGM12);  //Set for fast PWM, mode 14 (TOP = ICR1).PAGE 137
	TCCR1B |= (1 << WGM13);  //Set for fast PWM, mode 14 (TOP = ICR1).PAGE 137

	TCCR1A |= (1 << COM1A1);//For pin OC1A to be clear on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR1A &= ~(1 << COM1A0);//For pin OC1A to be clear on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR1A |= (1 << COM1B1);//For pin OC1B to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR1A &= ~(1 << COM1B0);//For pin OC1B to be set on compare match, FAST-PWM, inverting mode.PAGE 160


	TCCR1B |= (1 << CS10); // 1/1 PAGE 138
	TCCR1B &= ~(1 << CS11); // 1/1 PAGE 138
	TCCR1B &= ~(1 << CS12); // 1/1 PAGE 138

	ICR1H = (10000>>8);
	ICR1L = 10000;
	
	OCR1A = 100;

	DDRB |= 1 << DDB1; //Motor 2 control line, Timer2 PWM output B (OC2B)	
	DDRB |= 1 << DDB2; //Motor 2 control line, Timer2 PWM output A (OC2A)

}
*/
