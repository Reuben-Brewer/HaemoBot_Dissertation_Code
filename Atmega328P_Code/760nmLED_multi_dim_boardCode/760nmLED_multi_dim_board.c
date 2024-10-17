#define F_CPU 20000000UL	// Baby Orangutan frequency (20MHz), // F_CPU tells util/delay.h our clock frequency
#define SCL_CLOCK  100000L
#define dead_man_time 0.5 //500mS
#define dead_man_enable_flag_define 1 //Sets whether or not the board should use deadman timer.
#define TxFlag 1
#define send_every_byte_back_as_received_mode 0
//////////////////////////////////////////////

//////////////////////////////////////////////
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//////////////////////////////////////////////

//////////////////////////////////////////////
void PinInit(void);
void Motor0Init(void);
void Motor1Init(void);
void setMotor_0_char(unsigned char value);
void setMotor_1_char(unsigned char value);
void TimerOneInit(void);
void SerialInit(void);
unsigned int CheckAndWriteNewBooleanSerialValue(unsigned int oldVal, unsigned int newVal);
void setDebugLED_0(unsigned int value);
void setDebugLED_1(unsigned int value);
void toggleDebugLED_0(void);
void toggleDebugLED_1(void);
void getBoardnum(void);
//////////////////////////////////////////////

//////////////////////////////////////////////
volatile unsigned int TxCounter = 0;
volatile float main_loop_global_time = 0.0;
volatile unsigned int Rx_mutex_flag;
volatile float last_Rx_time = 0.0;
volatile float debug_to_computer;
volatile unsigned int counterRx = 0;
volatile unsigned int message_counter = 0;
volatile unsigned int message_length = 0;
volatile unsigned int RxMessage[26];
volatile unsigned int received_checksum, calculated_checksum;
volatile int message_being_processed = 0;
volatile unsigned char LEDbrightnessRx[20];
volatile unsigned char thisBoardLED_brightness_0 = 0;
volatile unsigned char thisBoardLED_brightness_1 = 0;
volatile unsigned int debugState0 = 0;
volatile unsigned int debugState1 = 0;
volatile unsigned char BoardNum_bit_0 = 0;
volatile unsigned char BoardNum_bit_1 = 0;
volatile unsigned char BoardNum_bit_2 = 0;
volatile unsigned char BoardNum_bit_3 = 0;
volatile unsigned char BoardNum = 77;
volatile unsigned char enableDMT = 0;
//////////////////////////////////////////////

//////////////////////////////////////////////
typedef union //c-syntax
{
	float float_num;
	char char_num[4];
} my_union; //c-syntax
//////////////////////////////////////////////

//////////////////////////////////////////////
inline int check_deadman_expired(void)
{
	if(main_loop_global_time - last_Rx_time < dead_man_time)
	{
		return 0; //DMT not expired
	}
	else
	{
		return 1; //DMT expired
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline int check_deadman_enabled(void)
{
	if(dead_man_enable_flag_define == 1)
	{
		return 1; //DMT enabled
	}
	else
	{
		return 0; //DMT disabled
	}		
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
ISR(USART_RX_vect)
{
	int headerCounter;
	unsigned char ledForCounter = 0;

	toggleDebugLED_0();
	if(send_every_byte_back_as_received_mode == 1)
	{
		send_serial_byte(UDR0);
	}
	else
	{
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
					message_length = 25;  //Message length counting message bytes from 0.
				}
				else if(RxMessage[4] == 2)
				{
					message_length = 25; //Message length counting message bytes from 0.
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
			

				if(RxMessage[4] == 1) // Motor position message
				{		
					//FF--00--FF--00--MESSAGE_NUM(1)--L0--L1--L2--L3--L4--L5--L6--L7--L8--L9--L10--L11--L12--L13--L14--L15--L16--L17--L18--L19--CHECKSUM
					//0   1   2   3   4               5   6   7   8   9   10  11  12  13  14  15   16   17   18   19   20   21   22   23   24   25
				
			
					Rx_mutex_flag = 1; ///////

					for(ledForCounter = 0; ledForCounter <= 19; ledForCounter++)
						{
							LEDbrightnessRx[ledForCounter] = RxMessage[5 + ledForCounter];
						}

					thisBoardLED_brightness_0 = LEDbrightnessRx[2*BoardNum];
					thisBoardLED_brightness_1 = LEDbrightnessRx[2*BoardNum + 1];


					received_checksum = RxMessage[7];

					Rx_mutex_flag = 0; ///////


					toggleDebugLED_1();

				}
				else if(RxMessage[4] == 2) //unused message as of yet
				{
				}

				last_Rx_time = main_loop_global_time;
				counterRx++;

			}	
		}
	}
}
//////////////////////////////////////////////


//////////////////////////////////////////////
int main(void) 
{	
	
	Motor0Init();
	Motor1Init();
	TimerOneInit();
	PinInit();
	SerialInit();
	sei();
	

	while(1)
	{	
		if ((TIFR1 & 0b00000010) != 0)
		{
			main_loop_global_time = main_loop_global_time + 0.001; //+ 1 mS

				if(Rx_mutex_flag == 0)
				{
					if(check_deadman_enabled() == 1)
					{
						if(check_deadman_expired() == 0) //if DTM is enabled 
						{
							setMotor_0_char(thisBoardLED_brightness_0); //and not-expired
							setMotor_1_char(thisBoardLED_brightness_1);	
						}
						else if(check_deadman_expired() == 1) //if DTM is enabled and expired
						{
							setMotor_0_char(0); //shut off LEDs if you no longer hear serial messages from the computer
							setMotor_1_char(0);	
						}
					}
					else
					{
						setMotor_0_char(thisBoardLED_brightness_0); //DMT not enabled
						setMotor_1_char(thisBoardLED_brightness_1);	
					}
				}

			TxCounter = TxCounter + 1;
			if(TxCounter == 200) //every 30ms
			{
				getBoardnum();
				if(TxFlag == 1)
				{
					send_serial_byte(0xFF); //byte 0
					send_serial_byte(0x00); //byte 1
					send_serial_byte(0xFF); //byte 2
					send_serial_byte(0x00); //byte 3
					send_serial_byte(0x01); //byte 4 Message Type 1
					send_serial_float(main_loop_global_time); //byte 5:8
						debug_to_computer = thisBoardLED_brightness_1; //main_loop_global_time - last_Rx_time;
					send_serial_float(debug_to_computer); //bytes 9:12 debug message
					send_serial_byte(77); //byte 13, checksum	
					toggleDebugLED_0();				
				}

				TxCounter = 0;
			}


			
			TIFR1 |= (1<<OCF1A); //clear interrupt flag
		}
	}

	return 0;
}
//////////////////////////////////////////////



//////////////////////////////////////////////
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
//////////////////////////////////////////////

//////////////////////////////////////////////
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
//////////////////////////////////////////////

//////////////////////////////////////////////
void setMotor_0_char(unsigned char value)
{
		OCR0A = 0;
		OCR0B = value; //Lights LED with positive terminal on Motor B terminal
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void setMotor_1_char(unsigned char value)
{
		OCR2A = 0;
		OCR2B = value; //Lights LED with positive terminal on Motor B terminal
}		
//////////////////////////////////////////////

//////////////////////////////////////////////
void PinInit(void)
{
	
	///////////////////////Input DI pins
	DDRC &= ~(1 << DDC0); // PORTC0 set low to be a digital input for BoardNum_bit_0.
	//PORTC |= (1 << PORTC0); //Turn on pull-up resistor.

	DDRC &= ~(1 << DDC1); // PORTC1 set low to be a digital input for BoardNum_bit_1.
	//PORTC |= (1 << PORTC1); //Turn on pull-up resistor.

	DDRC &= ~(1 << DDC2); // PORTC2 set low to be a digital input for BoardNum_bit_2.
	//PORTC |= (1 << PORTC2); //Turn on pull-up resistor.

	DDRC &= ~(1 << DDC3); // PORTC3 set low to be a digital input for BoardNum_bit_3.
	//PORTC |= (1 << PORTC3); //Turn on pull-up resistor.
	///////////////////////

	///////////////////////Output DO pins
	DDRD |= (1 << DDD4); //PORTD4 set hi to be a digital output for LED for debugging.
	DDRD |= (1 << DDD7); //PORTD7 set hi to be a digital output for LED for debugging. 
	///////////////////////
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void setDebugLED_0(unsigned int value)
{
	if(value == 1)
	{
		 PORTD |= (1 << PORTD4); //turn on user LED
	}
	else if(value == 0)
	{
		PORTD &= ~(1 << PORTD4); //turn off user LED
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void setDebugLED_1(unsigned int value)
{
	if(value == 1)
	{
		 PORTD |= (1 << PORTD7); //turn on user LED
	}
	else if(value == 0)
	{
		PORTD &= ~(1 << PORTD7); //turn off user LED
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void toggleDebugLED_0(void)
{
	if(debugState0 == 0)
	{
		debugState0 = 1;
		setDebugLED_0(1);
	}
	else if(debugState0 == 1)
	{
		debugState0 = 0;
		setDebugLED_0(0);
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void toggleDebugLED_1(void)
{
	if(debugState1 == 0)
	{
		debugState1 = 1;
		setDebugLED_1(1);
	}
	else if(debugState1 == 1)
	{
		debugState1 = 0;
		setDebugLED_1(0);
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void getBoardnum(void)
{
	if((0b00000001 & PINC) != 0) //PC0
	{
		BoardNum_bit_0 = 1; 
	}
	else
	{
		BoardNum_bit_0 = 0; 
	}	
	if((0b00000010 & PINC) != 0) //PC1
	{
		BoardNum_bit_1 = 1; 
	}
	else
	{
		BoardNum_bit_1 = 0; 
	}
	if((0b00000100 & PINC) != 0) //PC2
	{
		BoardNum_bit_2 = 1; 
	}
	else
	{
		BoardNum_bit_2 = 0; 
	}
	if((0b00001000 & PINC) != 0) //PC3
	{
		BoardNum_bit_3 = 1; 
	}
	else
	{
		BoardNum_bit_3 = 0; 
	}

	BoardNum = (BoardNum_bit_0 << 0) | (BoardNum_bit_1 << 1) | (BoardNum_bit_2 << 2) | (BoardNum_bit_3 << 3);
}
//////////////////////////////////////////////



