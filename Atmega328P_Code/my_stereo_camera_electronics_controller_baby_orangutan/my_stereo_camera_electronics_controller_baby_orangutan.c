#define F_CPU 20000000UL	// Baby Orangutan frequency (20MHz), // F_CPU tells util/delay.h our clock frequency
#define SCL_CLOCK  100000L
#define dead_man_time_counter_threshold 5000 //Number of times through main While(1) loop since we heard a Rx message.
#define dead_man_global_software_enabled 1
#define dead_man_FPS_software_enabled 0 //Controls only the FPS pin.
#define TxFlag 1
#define send_every_byte_back_as_received_mode 0
#define cameraTriggerHiPulseTime 0.005 //seconds, so this is 5mS
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
void setMotor_0_signed_int(signed int value);
void setMotor_1_signed_int(signed int value);
void TimerOneInit(void);
void SerialInit(void);
unsigned int CheckAndWriteNewBooleanSerialValue(unsigned int oldVal, unsigned int newVal);
void setLaser_0(unsigned int value);
void setLaser_1(unsigned int value);
void setCameraFPSpin(unsigned int value);
void setDebugLED_0(unsigned int value);
void setDebugLED_1(unsigned int value);
void toggleDebugLED_0(void);
void toggleDebugLED_1(void);
void toggleCameraFPSpin(void);
void getBoardnum(void);
void turnFPSpinOn(void);
void turnFPSpinOff(void);
//////////////////////////////////////////////

//////////////////////////////////////////////
volatile unsigned int TxCounter = 0;
volatile unsigned int TxMessageNum = 0;
volatile float main_loop_global_time = 0.0;
volatile float last_main_loop_global_time = 0.0;
volatile unsigned long int main_loop_counter = 0;
volatile unsigned long int main_loop_counter_at_last_Rx = 0;
volatile unsigned int Rx_mutex_flag;
volatile float last_Rx_time = 0.0;
volatile float last_Tx_time = 0.0;
volatile float last_camera_trigger_time = 0.0;
volatile float debug_to_computer;
volatile unsigned int counterRx = 0;
volatile unsigned int lastCounterRx = 0;
volatile unsigned int message_counter = 0;
volatile unsigned int message_length = 0;
volatile unsigned int RxMessage[13];
volatile unsigned int received_checksum, calculated_checksum;
volatile int message_being_processed = 0;
volatile unsigned int debugState0 = 0;
volatile unsigned int debugState1 = 0;
volatile unsigned int cameraFPSpinState = 0;
volatile unsigned char enableDMT = 0;
volatile unsigned int isFPSpinOn = 0;
volatile unsigned int cameraFPSblockSignal = 0;

volatile unsigned char cameraFPS = 2; //DEFAULT VALUE ON POWER UP
volatile unsigned char last_cameraFPS = 999;
volatile unsigned char laser0State = 0;
volatile unsigned int laser0SpeedLoByte = 0;
volatile unsigned int laser0SpeedHiByte = 0;
volatile signed int laser0Speed = 0;
volatile unsigned char laser1State = 0;
volatile unsigned int laser1SpeedLoByte = 0;
volatile unsigned int laser1SpeedHiByte = 0;
volatile signed int laser1Speed = 0;
volatile unsigned int FPS_register_value;
volatile unsigned int FPS_register_value_OCR1A;
volatile unsigned int FPS_register_value_OCR1B;
volatile unsigned int TimerOnePrescalerN;
volatile unsigned int NumRxMessageHasntChanged = 0;
//////////////////////////////////////////////

//////////////////////////////////////////////
typedef union //c-syntax
{
	float float_num;
	char char_num[4];
} my_union; //c-syntax
//////////////////////////////////////////////

//////////////////////////////////////////////
inline void turnFPSpinOn(void)
{
	cameraFPSblockSignal = 0;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline void turnFPSpinOff(void)
{
	cameraFPSblockSignal = 1;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline  void CalcAndSetTimer1registerFPS(void)
{
	FPS_register_value_OCR1A =  (float) cameraTriggerHiPulseTime*20000000.0/TimerOnePrescalerN;
	FPS_register_value_OCR1B =  (float) 20000000.0/(cameraFPS*TimerOnePrescalerN);


	//Have to disable and clear interrupts and TCNT1 so that we don't get delay in the new FPS values taking effect. 
	//Otherwise, we could set an OCR1A lower than the current TCNT1.
	TIMSK1 &= ~(1<<OCIE1A); //Disable OCR1A intterupts.
	TIMSK1 &= ~(1<<OCIE1B); //Disable OCR1B intterupts.	
	TIFR1 |= (1<<OCF1A); //Clear OCR1A interrupt flag
	TIFR1 |= (1<<OCF1B); //Clear OCR1B interrupt flag

	OCR1A = FPS_register_value_OCR1A;
	OCR1B = FPS_register_value_OCR1B;
	
	TCNT1 = 0; //Clear Timer One.
	TIMSK1 |= (1<<OCIE1A); //Enable OCR1A intterupts.
	TIMSK1 |= (1<<OCIE1B); //Enable OCR1B intterupts.	

	last_cameraFPS = cameraFPS;
}
//////////////////////////////////////////////



//////////////////////////////////////////////
inline  signed int reconstructSignedIntRxSerial(unsigned int HiByte, unsigned int LoByte)
{
	signed int reconstructed_value;
	if(HiByte == 255)
	{
		reconstructed_value = (signed int) 1*LoByte;
	}
	else
	{
		reconstructed_value = (signed int) -1*LoByte;
	}
	return reconstructed_value;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline int check_deadman_expired_counter_based(void)
{
	if(main_loop_counter - main_loop_counter_at_last_Rx < dead_man_time_counter_threshold)
	{
		return 0; //DMT has not expired.
	}
	else
	{
		return 1; //DMT has expired.
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline int check_deadman_enabled(void)
{
	if(dead_man_global_software_enabled == 1) 
	{
		return 1; //DMT enabled if dead_man_software_enabled == 1
	}
	else
	{
		return 0; //DMT disabled if dead_man_software_enabled == 0
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
ISR(TIMER1_COMPA_vect)
{
	cameraFPSpinState = 0;
	setCameraFPSpin(0);
	
	TIFR1 |= (1<<OCF1A); //Clear OCR1A interrupt flag
}
//////////////////////////////////////////////

//////////////////////////////////////////////
ISR(TIMER1_COMPB_vect)
{
	if(cameraFPSblockSignal == 0 && cameraFPS > 0) ///only pulses if the camera is supposed to be on and is non-zero FPS
	{
		cameraFPSpinState = 1;
		setCameraFPSpin(1);
	}

	TCNT1 = 0;
	TIFR1 |= (1<<OCF1B); //Clear OCR1B interrupt flag
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
					message_length = 12;  //Message length counting message bytes from 0.
				}
				else if(RxMessage[4] == 2)
				{
					message_length = 999; //Message length counting message bytes from 0.
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
			

				if(RxMessage[4] == 1) // set camera FPS, laser state, and laser speed message
				{		
					//FF--00--FF--00--MESSAGE_NUM(1)-cameraFPS--laser0State--laser0Speed_LObyte--laser0Speed_HIbyte--laser1State--laser1Speed_LObyte--laser1Speed_HIbyte--CHECKSUM
					//0   1   2   3   4              5          6            7                   8                   9            10                  11                  12 
				
			
					Rx_mutex_flag = 1; ///////

				 	cameraFPS =   RxMessage[5];

					laser0State = RxMessage[6];
					laser0SpeedLoByte = RxMessage[7];
					laser0SpeedHiByte = RxMessage[8];
				 	laser0Speed = reconstructSignedIntRxSerial(laser0SpeedHiByte, laser0SpeedLoByte);

					laser1State = RxMessage[9];
					laser1SpeedLoByte = RxMessage[10];
					laser1SpeedHiByte = RxMessage[11];
					laser1Speed = reconstructSignedIntRxSerial(laser1SpeedHiByte, laser1SpeedLoByte);
					
					received_checksum = RxMessage[12];

					Rx_mutex_flag = 0; ///////


					toggleDebugLED_1();

				}
				else if(RxMessage[4] == 2) //unused message as of yet
				{
				}

				last_Rx_time = main_loop_global_time;
				main_loop_counter_at_last_Rx = main_loop_counter;
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
				if(Rx_mutex_flag == 0)
				{
				
					if(cameraFPS != last_cameraFPS) //Only update the FPS-controlling OCR1A and OCR1B registers if we need to because the commanded FPS changed.
					{
						CalcAndSetTimer1registerFPS();
					}
					
					if(check_deadman_enabled() == 1)
					{
						if(check_deadman_expired_counter_based() == 0) //if DTM is enabled  and not-expired
						{
							setLaser_0(laser0State);
							setLaser_1(laser1State);
							setMotor_0_signed_int(laser0Speed); 
							setMotor_1_signed_int(laser1Speed);
							if(dead_man_FPS_software_enabled == 1)
							{
								turnFPSpinOn();
							}

						}
						else if(check_deadman_expired_counter_based() == 1) //if DTM is enabled and expired
						{
							setLaser_0(0); //turn off laser 0.
							setLaser_1(0);	//turn off laser 1.
							setMotor_0_signed_int(0); //shut off LEDs if you no longer hear serial messages from the computer
							setMotor_1_signed_int(0);
							if(dead_man_FPS_software_enabled == 1)
							{
								turnFPSpinOff();
							}
						}
					}
					else
					{
						setLaser_0(laser0State);
						setLaser_1(laser1State);
						setMotor_0_signed_int(laser0Speed); //DMT not enabled
						setMotor_1_signed_int(laser1Speed);
						if(dead_man_FPS_software_enabled == 1)
						{
							turnFPSpinOn();
						}
					}


					TxCounter = TxCounter + 1;

					if(TxCounter == 10000) 
					{
						TxMessageNum = TxMessageNum + 1;

						if(TxFlag == 1)
						{
							send_serial_byte(0xFF); //byte 0
							send_serial_byte(0x00); //byte 1
							send_serial_byte(0xFF); //byte 2
							send_serial_byte(0x00); //byte 3
							send_serial_byte(0x01); //byte 4 Message Type 1
							send_serial_float(main_loop_global_time); //byte 5:8
								debug_to_computer = (float) TxMessageNum;//main_loop_global_time - last_Rx_time;
							send_serial_float(debug_to_computer); //bytes 9:12 debug message
							send_serial_byte(77); //byte 13, checksum					
						}

						TxCounter = 0;
					}

				}//End bracket for if(Rx_mutex_flag == 0)

		main_loop_counter = main_loop_counter + 1;
	}

	return 0;
}
//////////////////////////////////////////////



//////////////////////////////////////////////
void TimerOneInit(void)
{

	TIMSK1 |= (1<<OCIE1A); //Enable OCR1A intterupts.
	TIMSK1 |= (1<<OCIE1B); //Enable OCR1B intterupts.

	TCCR1A &= ~(1<<COM1B0); //Normal operation, OCR1B Pin disconnected (page 135).
	TCCR1A &= ~(1<<COM1B1); //Normal operation, OCR1B Pin disconnected (page 135).

	
	TCCR1A &= ~(1<<WGM10); //WGM10 to 0 , mode 0 (TOP = FFFF). PAGE 137
	TCCR1A &= ~(1<<WGM11); //WGM11 to 0 , mode 0 (TOP = FFFF). PAGE 137
	TCCR1B &= ~(1 << WGM12); //WGM12 to 0 , mode 0 (TOP = FFFF). PAGE 137
	TCCR1B &= ~(1 << WGM13); //WGM12 to 0 , mode 0 (TOP = FFFF). PAGE 137

	TimerOnePrescalerN = 1024;
	TCCR1B |=  (1<<CS10); //1    Prescaler set to 1/1024th page 138 
	TCCR1B &= ~(1<<CS11); //0    Prescaler set to 1/1024th page 138
	TCCR1B |=  (1<<CS12); //1    Prescaler set to 1/1024th page 138

//	OCR1A = 97;
//	OCR1B = 651;
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
void setMotor_0_signed_int(signed int value)
{
	if(value >= 0)
	{
		OCR0A = 0;
		OCR0B = abs(value); //Lights LED with positive terminal on Motor B terminal
	}
	else
	{
		OCR0A = abs(value);
		OCR0B = 0; //Lights LED with positive terminal on Motor B terminal
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void setMotor_1_signed_int(signed int value)
{
	if(value >= 0)
	{
		OCR2A = 0;
		OCR2B = abs(value); //Lights LED with positive terminal on Motor B terminal
	}
	else
	{
		OCR2A = abs(value);
		OCR2B = 0; //Lights LED with positive terminal on Motor B terminal
	}
}		
//////////////////////////////////////////////

//////////////////////////////////////////////
void PinInit(void)
{
	///////////////////////Input DI pins
	///////////////////////

	///////////////////////Output DO pins
	DDRB |= (1 << DDB0); //PORTB0 set hi to be a digital output for controlling laser 0.
	DDRB |= (1 << DDB1); //PORTB1 set hi to be a digital output for controlling laser 1.
	DDRB |= (1 << DDB2); //PORTB2 set hi to be a digital output for camera FPS.
	isFPSpinOn = 1;

	DDRD |= (1 << DDD4); //PORTD4 set hi to be a digital output for LED for debugging.
	DDRD |= (1 << DDD7); //PORTD7 set hi to be a digital output for LED for debugging.
	///////////////////////
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void setLaser_0(unsigned int value)
{
	if(value == 1)
	{
		PORTB |= (1 << PORTB0); //turn on laser 0.
	}
	else if(value == 0)
	{
		PORTB &= ~(1 << PORTB0); //turn off laser 0.
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void setLaser_1(unsigned int value)
{
	if(value == 1)
	{
		PORTB |= (1 << PORTB1); //turn on laser 1.
	}
	else if(value == 0)
	{
		PORTB &= ~(1 << PORTB1); //turn off laser 1.
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void setCameraFPSpin(unsigned int value)
{
	if(value == 1)
	{
		PORTB |= (1 << PORTB2); //turn on camera FPS pin.
	}
	else if(value == 0)
	{
		PORTB &= ~(1 << PORTB2); //turn off camera FPS pin.
	}
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
void toggleCameraFPSpin(void)
{
	if(cameraFPSpinState == 0)
	{
		cameraFPSpinState = 1;
		setCameraFPSpin(1);
	}
	else if(cameraFPSpinState == 1)
	{
		cameraFPSpinState = 0;
		setCameraFPSpin(0);
	}
}
//////////////////////////////////////////////




/*

//	TIMSK1 |= (1<<OCIE1A); //Enable OCR1A intterupts.
//	TIMSK1 |= (1<<OCIE1B); //Enable OCR1B intterupts.

//	TCCR1A &= ~(1<<COM1B0); //Normal operation, OCR1B Pin disconnected (page 135).
//	TCCR1A &= ~(1<<COM1B1); //Normal operation, OCR1B Pin disconnected (page 135).

//	TCCR1A |= (1<<COM1B0); //Toggle OCR1B Pin on compare match (page 135).
//	TCCR1A &= ~(1<<COM1B1); //Toggle OCR1B Pin on compare match (page 135). 

	TCCR1A &= ~(1<<COM1B0); //Clear OCR1B Pin on compare match (page 135).
	TCCR1A |= (1<<COM1B1); //Clear OCR1B Pin on compare match (page 135). 

//	TCCR1A |= (1<<COM1B0); //Set OCR1B Pin on compare match (page 135).
//	TCCR1A |= (1<<COM1B1); //Set OCR1B Pin on compare match (page 135). 


	TCCR1A &= ~(1<<WGM10); //WGM10 to 0 for CTC (clear timer on compare), mode 4 (TOP = OCR1A). PAGE 137
	TCCR1A &= ~(1<<WGM11); //WGM11 to 0 for CTC (clear timer on compare), mode 4 (TOP = OCR1A). PAGE 137
	TCCR1B |= (1 << WGM12); //WGM12 to 1 for CTC (clear timer on compare), mode 4 (TOP = OCR1A). PAGE 137
	TCCR1B &= ~(1 << WGM13); //WGM12 to 0 for CTC (clear timer on compare), mode 4 (TOP = OCR1A). PAGE 137

//	TCCR1A &= ~(1<<WGM10); //WGM10 to 0 , mode 11 (TOP = OCR1A). PAGE 137
//	TCCR1A |= (1<<WGM11); //WGM11 to 1 , mode 11 (TOP = OCR1A). PAGE 137
//	TCCR1B &= ~(1 << WGM12); //WGM12 to 0 , mode 11 (TOP = OCR1A). PAGE 137
//	TCCR1B |= (1 << WGM13); //WGM12 to 1 , mode 11 (TOP = OCR1A). PAGE 137
	
//	TCCR1A &= ~(1<<WGM10); //WGM10 to 0 , mode 0 (TOP = FFFF). PAGE 137
//	TCCR1A &= ~(1<<WGM11); //WGM11 to 0 , mode 0 (TOP = FFFF). PAGE 137
//	TCCR1B &= ~(1 << WGM12); //WGM12 to 0 , mode 0 (TOP = FFFF). PAGE 137
//	TCCR1B &= ~(1 << WGM13); //WGM12 to 0 , mode 0 (TOP = FFFF). PAGE 137

	TimerOnePrescalerN = 1024;
	TCCR1B |=  (1<<CS10); //1    Prescaler set to 1/1024th page 138 /////////freq_timer1=20e6/(2*N_clock_prescaler*(1+OCR1A)), page 127///////////
	TCCR1B &= ~(1<<CS11); //0    Prescaler set to 1/1024th page 138
	TCCR1B |=  (1<<CS12); //1    Prescaler set to 1/1024th page 138




*/
