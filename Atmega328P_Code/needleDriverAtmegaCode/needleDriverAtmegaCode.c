#define F_CPU 20000000UL	// Baby Orangutan frequency (20MHz), // F_CPU tells util/delay.h our clock frequency
#define SCL_CLOCK  100000L
#define dead_man_enable_flag_define 1 //Sets whether or not the 
#define dead_man_time 0.125 //seconds in float (include the decimal)
#define TxFlag 1

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void setDebugLED_0(unsigned int value);
void toggleDebugLED_0(void);
void I2C_readAllAxes(void);
void I2C_init(void);
void I2C_set_start(void);
void I2C_clear_start(void);
void I2C_stop(void);
void I2C_enable_ACK(void);
void I2C_disable_ACK(void);
void initAccel(void);
void readAccel(void);
unsigned char I2C_read(unsigned char register_address);
void I2C_write(unsigned char register_address, unsigned char data_byte);
void Motor0Init(void);
void Motor1Init(void);
void ADCInit(void);
void ADCSetChannel(unsigned char channel);
void TouchSensorInit(void);
void TimerOneInit(void);
void SerialInit(void);
void GetTouchState(void);
void laserInit(void);
void setLaser(void);
unsigned int CheckAndWriteNewBooleanSerialValue(unsigned int oldVal, unsigned int newVal);

volatile unsigned int ADCcounter;
volatile unsigned char ADCloByte;
volatile unsigned char ADChiByte;
volatile unsigned int analogSignal = 0;
volatile unsigned int analogSignal0 = 0;
volatile unsigned int analogSignal1 = 0;
volatile unsigned int analogSignal2 = 0;
volatile unsigned int analogSignal3 = 0;
volatile unsigned int analogSignal4 = 0;
volatile unsigned int analogThreshold0 = 350; //500 motor0_limit_switch[0] LOOP OPEN (FRONT) POSITION
volatile unsigned int analogThreshold1 = 350; //500 motor0_limit_switch[1] LOOP CLOSED (BACK) POSITION
volatile unsigned int analogThreshold2 = 500; //500 motor1_limit_switch[0] SWING ARM OPEN (FRONT) POSITION
volatile unsigned int analogThreshold3 = 350; //500 motor1_limit_switch[1] SWING ARM CLOSED (BACK) POSITION
volatile unsigned int analogThreshold4 = 500; //500 cathHomeSwitch
volatile unsigned int touchState = 0;
volatile unsigned int laserState = 0;
volatile unsigned int TxCounter = 0;
volatile float main_loop_global_time = 0.0;
volatile float current_loop_time = 0.0;
volatile float last_loop_time = 0.0;
volatile unsigned int debugState0 = 0;
volatile unsigned int DMT_is_expired_state = 0;
volatile unsigned int last_DMT_is_expired_state = 0;
volatile unsigned int DMT_is_expired_state_HAS_CHANGED = 0;


unsigned char I2Clowbyte = 0;
volatile unsigned int Rx_mutex_flag;
volatile float last_Rx_time = 0.0;
volatile float time_between_Rx_messages = 0.0;
volatile float debug_to_computer;
volatile unsigned int counterRx = 0;
volatile unsigned int message_counter = 0;
volatile unsigned int message_length = 0;
volatile unsigned int RxMessage[11];
volatile unsigned int received_checksum, calculated_checksum;
volatile int message_being_processed = 0;
volatile unsigned int motor_goal_pos[2];
volatile unsigned int last_motor_goal_pos[2];
volatile unsigned int motor_goal_vel[2];
volatile unsigned int last_motor_goal_vel[2];
volatile unsigned int motor0_limit_switch[2];
volatile unsigned int motor1_limit_switch[2];
volatile unsigned int last_motor0_limit_switch[2];
volatile unsigned int last_motor1_limit_switch[2];
volatile unsigned int motor0_limit_switch_HAS_CHANGED = 0;
volatile unsigned int motor1_limit_switch_HAS_CHANGED = 0;
volatile unsigned int motor0_command_HAS_CHANGED = 0;
volatile unsigned int motor1_command_HAS_CHANGED = 0;
volatile unsigned int cathHomeSwitch = 0;
volatile unsigned int AxLo, AxHi, AyLo, AyHi,AzLo, AzHi;
volatile signed int Ax10Bit, Ay10Bit, Az10Bit;

#define I2CdebugOverSerialFlag 0
#define numADCchannels 5

#define ADXL345_write_address 0xa6 
#define ADXL345_read_address 0xa7 
#define ADXL345_DEVID 0x00
#define ADXL345_RESERVED1 0x01
#define ADXL345_THRESH_TAP 0x1d
#define ADXL345_OFSX 0x1e
#define ADXL345_OFSY 0x1f
#define ADXL345_OFSZ 0x20
#define ADXL345_DUR 0x21
#define ADXL345_LATENT 0x22
#define ADXL345_WINDOW 0x23
#define ADXL345_THRESH_ACT 0x24
#define ADXL345_THRESH_INACT 0x25
#define ADXL345_TIME_INACT 0x26
#define ADXL345_ACT_INACT_CTL 0x27
#define ADXL345_THRESH_FF 0x28
#define ADXL345_TIME_FF 0x29
#define ADXL345_TAP_AXES 0x2a
#define ADXL345_ACT_TAP_STATUS 0x2b
#define ADXL345_BW_RATE 0x2c
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_INT_ENABLE 0x2e
#define ADXL345_INT_MAP 0x2f
#define ADXL345_INT_SOURCE 0x30
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37
#define ADXL345_FIFO_CTL 0x38
#define ADXL345_FIFO_STATUS 0x39
#define ADXL345_BW_1600 0xF // 1111
#define ADXL345_BW_800  0xE // 1110
#define ADXL345_BW_400  0xD // 1101  
#define ADXL345_BW_200  0xC // 1100
#define ADXL345_BW_100  0xB // 1011  
#define ADXL345_BW_50   0xA // 1010 
#define ADXL345_BW_25   0x9 // 1001 
#define ADXL345_BW_12   0x8 // 1000 
#define ADXL345_BW_6    0x7 // 0111 
#define ADXL345_BW_3    0x6 // 0110  
#define masterTx_ACK 0x18
#define masterRx_ACK 0x40

typedef union //c-syntax
{
	float float_num;
	char char_num[4];
} my_union; //c-syntax

//////////////////////////////////////////////
inline int check_deadman_expired(void)
{
	time_between_Rx_messages = main_loop_global_time - last_Rx_time;
	if(time_between_Rx_messages < dead_man_time)
	{
		DMT_is_expired_state = 0;
	}
	else
	{
		DMT_is_expired_state = 1;

	}

	if(DMT_is_expired_state != last_DMT_is_expired_state)
	{
		DMT_is_expired_state_HAS_CHANGED = 1;
	}
	last_DMT_is_expired_state = DMT_is_expired_state;

	return DMT_is_expired_state;
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
inline void setDebugLED_0(unsigned int value)
{
	if(value == 1)
	{
		 PORTB |= (1 << PORTB0); //turn on LED
	}
	else if(value == 0)
	{
		 PORTB &= ~(1 << PORTB0); //turn off LED
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline void toggleDebugLED_0(void)
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

inline void setMotor0_on_full_speed()
{
	OCR0A = 255;
	OCR0B = 0;
}

inline void setMotor1_on_full_speed()
{
	OCR2A = 255;
	OCR2B = 0;
}

inline void setMotor0_off()
{
	OCR0A = 0;
	OCR0B = 0;
}

inline void setMotor1_off()
{
	OCR2A = 0;
	OCR2B = 0;
}

//////////////////////////////////////////////
inline void setMotor(int motor_num)
{
		if(motor_num == 0)
		{
			if(motor_goal_pos[0] == 0 && motor0_limit_switch[0] == 0)
			{
				OCR0A = 0;
				OCR0B = motor_goal_vel[0];
			}
			else if(motor_goal_pos[0] == 1 && motor0_limit_switch[1] == 0)
			{
				OCR0A = motor_goal_vel[0];
				OCR0B = 0;
			}
			else
			{
				OCR0A = 0;
				OCR0B = 0;
			}
		}
			
		else if(motor_num == 1)
		{
			if(motor_goal_pos[1] == 0 && motor1_limit_switch[0] == 0 )
			{
				OCR2A = motor_goal_vel[1];
				OCR2B = 0;
			}
			else if(motor_goal_pos[1] == 1 && motor1_limit_switch[1] == 0)
			{
				OCR2A = 0;
				OCR2B = motor_goal_vel[1];
			}
			else
			{
				OCR2A = 0;
				OCR2B = 0;
			}
		}

/*	//////////////////////////////////////////////
	if(check_deadman_enabled() == 1)
	{				
		if(check_deadman_expired() == 0) //The deadman IS enabled, and it hasn't expired, so set properly.
		{
			setDebugLED_0(0);
			if(motor_num == 0)
			{
				if(motor_goal_pos[0] == 0 && motor0_limit_switch[0] == 0)
				{
					OCR0A = 0;
					OCR0B = motor_goal_vel[0];
				}
				else if(motor_goal_pos[0] == 1 && motor0_limit_switch[1] == 0)
				{
					OCR0A = motor_goal_vel[0];
					OCR0B = 0;
				}
				else
				{
					OCR0A = 0;
					OCR0B = 0;
				}
			}

			else if(motor_num == 1)
			{
				if(motor_goal_pos[1] == 0 && motor1_limit_switch[0] == 0 )
				{
					OCR2A = 0;
					OCR2B = motor_goal_vel[1];
				}
				else if(motor_goal_pos[1] == 1 && motor1_limit_switch[1] == 0)
				{
					OCR2A = motor_goal_vel[1];
					OCR2B = 0;
				}
				else
				{
					OCR2A = 0;
					OCR2B = 0;
				}
			}
		}
		//////////////////////////////////////////////

		//////////////////////////////////////////////
		else if(check_deadman_expired() == 1)//The deadman IS enabled, and it HAS expired, so set all zeros.
		{
			setDebugLED_0(1);
			OCR0A = 0;
			OCR0B = 0;
			OCR2A = 0;
			OCR2B = 0;
		}
		//////////////////////////////////////////////
		
	}
	//////////////////////////////////////////////
	else if(check_deadman_enabled() == 0)//The deadman is NOT enabled, so set properly.
	{
		if(motor_num == 0)
		{
			if(motor_goal_pos[0] == 0 && motor0_limit_switch[0] == 0)
			{
				OCR0A = 0;
				OCR0B = motor_goal_vel[0];
			}
			else if(motor_goal_pos[0] == 1 && motor0_limit_switch[1] == 0)
			{
				OCR0A = motor_goal_vel[0];
				OCR0B = 0;
			}
			else
			{
				OCR0A = 0;
				OCR0B = 0;
			}
		}
			
		else if(motor_num == 1)
		{
			if(motor_goal_pos[1] == 0 && motor1_limit_switch[0] == 0 )
			{
				OCR2A = 0;
				OCR2B = motor_goal_vel[1];
			}
			else if(motor_goal_pos[1] == 1 && motor1_limit_switch[1] == 0)
			{
				OCR2A = motor_goal_vel[1];
				OCR2B = 0;
			}
			else
			{
				OCR2A = 0;
				OCR2B = 0;
			}
		}
	}
	//////////////////////////////////////////////*/
}
//////////////////////////////////////////////

inline void ADCReadAndRespond(unsigned char channel)
{

	ADCloByte = ADCL;
	ADChiByte	= ADCH;
	analogSignal = (ADCloByte |(ADChiByte<<8));

	//Put the A to D converted value into an integer (it is currently split in two registers)
	if(channel == 0)
	{
		analogSignal0 = analogSignal;
		if(analogSignal0 >= analogThreshold0)
		{
			motor0_limit_switch[0] = 1;
			PORTB |= (1 << PORTB2);
		}
		else
		{
			motor0_limit_switch[0] = 0;
			PORTB &= ~(1 << PORTB2);
		}

		if(motor0_limit_switch[0] != last_motor0_limit_switch[0])
		{
			motor0_limit_switch_HAS_CHANGED = 1;
		}
		last_motor0_limit_switch[0] = motor0_limit_switch[0];
	}
	else if(channel == 1)
	{
		
		analogSignal1 = analogSignal;
		if(analogSignal1 >= analogThreshold1)
		{
			motor0_limit_switch[1] = 1;
			PORTB |= (1 << PORTB1);
		}
		else
		{
			motor0_limit_switch[1] = 0;
			PORTB &= ~(1 << PORTB1);
		}

		if(motor0_limit_switch[1] != last_motor0_limit_switch[1])
		{
			motor0_limit_switch_HAS_CHANGED = 1;
		}
		last_motor0_limit_switch[1] = motor0_limit_switch[1];
	}
	else if(channel == 2)
	{
		
		analogSignal2 = analogSignal;
		if(analogSignal2 >= analogThreshold2)
		{
			motor1_limit_switch[0] = 1;
			PORTB |= (1 << PORTB4);
		}
		else
		{
			motor1_limit_switch[0] = 0;
			PORTB &= ~(1 << PORTB4);
		}

		if(motor1_limit_switch[0] != last_motor1_limit_switch[0])
		{
			motor1_limit_switch_HAS_CHANGED = 1;
		}
		last_motor1_limit_switch[0] = motor1_limit_switch[0];
	}
	else if(channel == 3)
	{
		
		analogSignal3 = analogSignal;
		
		if(analogSignal3 >= analogThreshold3)
		{
			motor1_limit_switch[1] = 1;
			PORTB |= (1 << PORTB5);
		}
		else
		{
			motor1_limit_switch[1] = 0;
			PORTB &= ~(1 << PORTB5);
		}

		if(motor1_limit_switch[1] != last_motor1_limit_switch[1])
		{
			motor1_limit_switch_HAS_CHANGED = 1;
		}
		last_motor1_limit_switch[1] = motor1_limit_switch[1];
	}
	else if(channel == 4)
	{
		
		analogSignal4 = analogSignal;
		
		if(analogSignal4 >= analogThreshold4)
		{
			cathHomeSwitch = 1;
			//PORTB |= (1 << PORTB0); 
		}
		else
		{
			cathHomeSwitch = 0;
			//PORTB &= ~(1 << PORTB0);
		}

	}
	
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
				message_length = 10;  //Message length counting message bytes from 0.
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
			

			if(RxMessage[4] == 1) // Motor position message
			{		
				//FF--00--FF--00--MESSAGE_NUM(1)--M0_POS_TARGET--M0_SPEED--M1_POS_TARGET--M1_SPEED--laserState--CHECKSUM
				//0   1   2   3   4               5              6         7              8         9           10
				
				//toggleDebugLED_0();
				Rx_mutex_flag = 1;
				motor_goal_pos[0] = CheckAndWriteNewBooleanSerialValue(motor_goal_pos[0], RxMessage[5]);
				motor_goal_vel[0] = RxMessage[6];
				motor_goal_pos[1] = CheckAndWriteNewBooleanSerialValue(motor_goal_pos[1], RxMessage[7]);
				motor_goal_vel[1] = RxMessage[8];
				laserState = CheckAndWriteNewBooleanSerialValue(laserState, RxMessage[9]);
				received_checksum = RxMessage[10];

				if(motor_goal_pos[0] != last_motor_goal_pos[0])
				{
					motor0_command_HAS_CHANGED = 1;
				}
				if(motor_goal_pos[1] != last_motor_goal_pos[1])
				{
					motor1_command_HAS_CHANGED = 1;
				}
				if(motor_goal_vel[0] != last_motor_goal_vel[0])
				{
					motor0_command_HAS_CHANGED = 1;
				}
				if(motor_goal_vel[1] != last_motor_goal_vel[1])
				{
					motor1_command_HAS_CHANGED = 1;
				}

				last_motor_goal_pos[0] = motor_goal_pos[0];
				last_motor_goal_pos[1] = motor_goal_pos[1];
				last_motor_goal_vel[0] = motor_goal_vel[0];
				last_motor_goal_vel[1] = motor_goal_vel[1];

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

	if(ADCcounter < numADCchannels - 1)
	{
		ADCcounter = ADCcounter + 1;
	}
	else
	{
		ADCcounter = 0;
	}

	ADCSetChannel(ADCcounter);

	ADCSRA |= (1 << ADIF); //Clear the A to D conversion complete flag (done by setting it to 1)
	ADCSRA |= (1 << ADIE); //Enables ADC interrupts.
	ADCSRA |= (1 << ADSC); //Initialize the next A to D conversion
}

int main(void) 
{
	
	motor_goal_pos[0] = 0;
	motor_goal_pos[1] = 0;

	last_motor_goal_pos[0] = 0;
	last_motor_goal_pos[1] = 0;

	motor_goal_vel[0] = 0;
	motor_goal_vel[1] = 0;

	last_motor_goal_vel[0] = 0;
	last_motor_goal_vel[1] = 0;

	motor0_limit_switch[0] = 0;
	motor0_limit_switch[1] = 0;
	motor1_limit_switch[0] = 0;
	motor1_limit_switch[1] = 0;

	last_motor0_limit_switch[0] = 0;
	last_motor0_limit_switch[1] = 0;
	last_motor1_limit_switch[0] = 0;
	last_motor1_limit_switch[1] = 0;
	
	Motor0Init();
	Motor1Init();
	ADCInit();
	TimerOneInit();
	SerialInit();
	I2C_init();
	initAccel();
	sei();

	//setMotor0_on_full_speed();
	//setMotor1_on_full_speed();

	while(1)
	{
		if ((TIFR1 & 0b00000010) != 0)
		{
			main_loop_global_time = main_loop_global_time + 0.001; //+ 1 mS
			current_loop_time = current_loop_time + 0.001; //+ 1 mS

			if(current_loop_time >= 0.002) //10mS
			{
				if(Rx_mutex_flag == 0)
				{
					GetTouchState();
					setLaser();

					if(check_deadman_enabled() == 1 && check_deadman_expired() == 1)
					{
						laserState = 0;
						setLaser();
						setDebugLED_0(1);
						setMotor0_off();
						setMotor1_off();
					}
					else
					{
						if(DMT_is_expired_state_HAS_CHANGED == 1)
						{
							motor0_command_HAS_CHANGED = 1;
							motor1_command_HAS_CHANGED = 1;
						}

						if(motor0_limit_switch_HAS_CHANGED == 1 || motor0_command_HAS_CHANGED == 1)
						{
							setMotor(0);
							motor0_limit_switch_HAS_CHANGED = 0;
							motor0_command_HAS_CHANGED = 0;
						}
					
						if(motor1_limit_switch_HAS_CHANGED == 1 || motor1_command_HAS_CHANGED == 1)
						{
							setMotor(1);
							motor1_limit_switch_HAS_CHANGED = 0;
							motor1_command_HAS_CHANGED = 0;
						}

						DMT_is_expired_state_HAS_CHANGED = 0;
					}  
				}

			current_loop_time = 0.0;
			toggleDebugLED_0();

			}
			
			


			TxCounter = TxCounter + 1;
			if(TxCounter == 20) //every 20ms
			{
				I2C_readAllAxes();
				//readAccel();

				if(TxFlag == 1)
				{
					send_serial_byte(0xFF); //byte 0
					send_serial_byte(0x00); //byte 1
					send_serial_byte(0xFF); //byte 2
					send_serial_byte(0x00); //byte 3
					send_serial_byte(0x01); //byte 4 Message Type 1
					send_serial_float(main_loop_global_time); //byte 5:8
					send_serial_byte((unsigned char) touchState); //byte 9
					send_serial_byte((unsigned char) motor0_limit_switch[0]); //byte 10
					send_serial_byte((unsigned char) motor0_limit_switch[1]); //byte 11
					send_serial_byte((unsigned char) motor1_limit_switch[0]); //byte 12
					send_serial_byte((unsigned char) motor1_limit_switch[1]); //byte 13
					send_serial_byte((unsigned char) cathHomeSwitch); //byte 14
					send_serial_int(Ax10Bit); //bytes 15:16 
					send_serial_int(Ay10Bit); //bytes 17:18 
					send_serial_int(Az10Bit); //bytes 19:20
	
					float temp = main_loop_global_time - last_Rx_time;
					//debug_to_computer = motor_goal_vel[0];

					//debug_to_computer = time_between_Rx_messages;

					debug_to_computer = analogSignal0;

					//debug_to_computer = counterRx;
					//debug_to_computer = motor_goal_pos[0];
					//debug_to_computer  = 777;
					send_serial_float(debug_to_computer); //bytes 21:24 debug message

					send_serial_byte(77); //byte 25, checksum					
				}

				TxCounter = 0;
			}

}
			
			TIFR1 |= (1<<OCF1A); //clear interrupt flag
		
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

void Motor1Init(void)
{
	TCCR2A |= (1 << WGM21);  //Set for fast PWM, mode 3 (TOP = 0xFF).PAGE 161
	TCCR2A |= (1 << WGM20);  //Set for fast PWM, mode (TOP = 0xFF).PAGE 161

	TCCR2A |= (1 << COM2A1);//For pin OC1A to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR2A |= (1 << COM2A0);//For pin OC1A to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR2A |= (1 << COM2B1);//For pin OC1B to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR2A |= (1 << COM2B0);//For pin OC1B to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	
	TCCR2B &= (1 << WGM22); //Set WGM2 to 0 for fast PWM, MODE 3 (TOP = 0xFF). PAGE 161

	TCCR2B &= ~(1 << CS20); // 1/8 PAGE 163
	TCCR2B |= (1 << CS21);  // 1/8 PAGE 163
	TCCR2B &= ~(1 << CS22); // 1/8 PAGE 163

	OCR2A = 0;
	OCR2B = 0;

	DDRD |= 1 << DDD3; //Motor 2 control line, Timer2 PWM output B (OC2B)	
	DDRB |= 1 << DDB3; //Motor 2 control line, Timer2 PWM output A (OC2A)
}

void ADCInit(void)
{

	ADCSRA |= (1 << ADPS2) | (1 << ADPS1)|(1 << ADPS0); //Sets the AD prescale to 1/128 or 156.25kHz
	ADCSRA |= (1 << ADEN); //Enables ADC.

	ADCcounter = 0;
    ADCSetChannel(ADCcounter); //must jump start the conversion so that we have a flag to look for in the main() loop

	DDRB |= 1 << DDB1; //PORTB1 is set as output for telling if motor0, limit switch 0 has been reached.
	DDRB |= 1 << DDB2; //PORTB2 is set as output for telling if motor0, limit switch 1 has been reached.	
	DDRB |= 1 << DDB4; //PORTB4 is set as output for telling if motor1, limit switch 0 has been reached.	
	DDRB |= 1 << DDB5; //PORTB5 is set as output for telling if motor1, limit switch 1 has been reached.
	DDRB |= 1 << DDB0; //PORTB0 is set as output for debug LED	

	DDRC &= ~(1 << DDC0); // PORTC0 set low to be an analog input.
	DDRC &= ~(1 << DDC1); // PORTC1 set low to be an analog input.
	DDRC &= ~(1 << DDC2); // PORTC2 set low to be an analog input.
	DDRC &= ~(1 << DDC3); // PORTC3 set low to be an analog input.
	DDRC &= ~(1 << DDC6); // ADC6 set low to be an analog input.

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

void GetTouchState(void)
{
	if((0b00000100 & PIND) != 0)
	{
		touchState = 0;
		PORTD &= ~(1 << PORTD7);
	}
	else
	{
		touchState = 1;
		PORTD |= (1 << PORTD7);
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


void laserInit(void)
{
	laserState = 0;
	DDRD |= (1 << PORTD4);
	setLaser();
}

void setLaser(void)
{
	if(laserState == 0)
	{
		PORTD |= (1 << PORTD4);
	}
	else if(laserState == 1)
	{
		PORTD &= ~(1 << PORTD4);
	}
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

void I2C_init(void)
{
	TWSR  &= ~(1 << TWPS0); //prescalar of 1
	TWSR  &= ~(1 << TWPS1); //prescalar of 1
  	TWBR = 92;//((F_CPU/SCL_CLOCK)-16)/2;  //sets SCL frequency.  /* must be > 10 for stable operation */
	TWCR |= (1 << TWEN); //Enables Two Wire Mode
	//TWCR |= (1 << TWIE); //Enables interrupt for Two Wire Mode whenever TWINT bit is raised on TWCR register

}

unsigned char I2C_read(unsigned char register_address)
{
	unsigned char rx_data;
	
	
	/////////////////////////In Master Tx (Transmitter) Mode
		if(I2CdebugOverSerialFlag) send_serial_byte(0x12);
	I2C_enable_ACK();
	I2C_set_start();
	
	if(I2CdebugOverSerialFlag) send_serial_byte(TWSR & 0xfc);
	TWDR = ADXL345_write_address;
	
	I2C_clear_start();

		if(I2CdebugOverSerialFlag) send_serial_byte(TWSR & 0xfc); 

	TWDR = register_address; //data register for Two Wire
	TWCR |= (1 << TWINT); //CLEAR interrupt by writing value to high
	while(!(TWCR & (1<<TWINT))); //Wait for byte to be sent before returning
		if(I2CdebugOverSerialFlag) send_serial_byte(TWSR & 0xfc);
	

	/////////////////////////In Master Rx (Receiver) Mode
	I2C_set_start();

	TWDR = ADXL345_read_address;

	I2C_clear_start();

		if(I2CdebugOverSerialFlag) send_serial_byte(TWSR & 0xfc);

	I2C_disable_ACK();
	TWCR |= (1 << TWINT); //CLEAR interrupt by writing value to high
	while(!(TWCR & (1<<TWINT))); //Wait for byte to be sent before returning
	rx_data = TWDR;
		if(I2CdebugOverSerialFlag) send_serial_byte(TWSR & 0xfc); 
		if(I2CdebugOverSerialFlag) send_serial_byte(rx_data);
	I2C_stop();

		if(I2CdebugOverSerialFlag) send_serial_byte(0xff);
	return rx_data;
}

void I2C_readAllAxes(void)
{	
	/////////////////////////In Master Tx (Transmitter) Mode
		if(I2CdebugOverSerialFlag) send_serial_byte(0x12);
	I2C_enable_ACK();
	I2C_set_start();
	
	if(I2CdebugOverSerialFlag) send_serial_byte(TWSR & 0xfc);
	TWDR = ADXL345_write_address;
	
	I2C_clear_start();

		if(I2CdebugOverSerialFlag) send_serial_byte(TWSR & 0xfc); 

	TWDR = ADXL345_DATAX0; //data register for Two Wire, start at X0, and we'll keep reading to the rest of the axes after this
	TWCR |= (1 << TWINT); //CLEAR interrupt by writing value to high
	while(!(TWCR & (1<<TWINT))); //Wait for byte to be sent before returning
		if(I2CdebugOverSerialFlag) send_serial_byte(TWSR & 0xfc);
	

	/////////////////////////In Master Rx (Receiver) Mode
	I2C_set_start();

	TWDR = ADXL345_read_address;

	I2C_clear_start();

		if(I2CdebugOverSerialFlag) send_serial_byte(TWSR & 0xfc);

	
	TWCR |= (1 << TWINT); //CLEAR interrupt by writing value to high
	while(!(TWCR & (1<<TWINT))); //Wait for byte to be sent before returning
	
	AxLo = TWDR;
	if(I2CdebugOverSerialFlag) send_serial_byte(AxLo);
	
	TWCR |= (1 << TWINT); //CLEAR interrupt by writing value to high
	while(!(TWCR & (1<<TWINT)));

	AxHi = TWDR;
	if(I2CdebugOverSerialFlag) send_serial_byte(AxHi);

	TWCR |= (1 << TWINT); //CLEAR interrupt by writing value to high
	while(!(TWCR & (1<<TWINT)));

	AyLo = TWDR;
	if(I2CdebugOverSerialFlag) send_serial_byte(AyLo);

	TWCR |= (1 << TWINT); //CLEAR interrupt by writing value to high
	while(!(TWCR & (1<<TWINT)));

	AyHi = TWDR;
	if(I2CdebugOverSerialFlag) send_serial_byte(AyHi);

	TWCR |= (1 << TWINT); //CLEAR interrupt by writing value to high
	while(!(TWCR & (1<<TWINT)));

	AzLo = TWDR;
	if(I2CdebugOverSerialFlag) send_serial_byte(AzLo);

	I2C_disable_ACK();	
	TWCR |= (1 << TWINT); //CLEAR interrupt by writing value to high

	TWCR |= (1 << TWINT); //CLEAR interrupt by writing value to high
	while(!(TWCR & (1<<TWINT)));



	AzHi = TWDR;
	if(I2CdebugOverSerialFlag) send_serial_byte(AzHi);


		if(I2CdebugOverSerialFlag) send_serial_byte(TWSR & 0xfc); 
	I2C_stop();

		if(I2CdebugOverSerialFlag) send_serial_byte(0xff);

	Ax10Bit = (signed int) (AxLo | (AxHi << 8));
	Ay10Bit = (signed int) (AyLo | (AyHi << 8));
	Az10Bit = (signed int) (AzLo | (AzHi << 8));
}


void I2C_write(unsigned char register_address, unsigned char data_byte)
{
	I2C_enable_ACK();
	
		if(I2CdebugOverSerialFlag) send_serial_byte(0x12);
	I2C_set_start();
	
		if(I2CdebugOverSerialFlag) send_serial_byte(TWSR & 0xfc);
	TWDR = ADXL345_write_address; //PAGE 224
	
	I2C_clear_start();


		if(I2CdebugOverSerialFlag) send_serial_byte(TWSR & 0xfc); 

	TWDR = register_address; //data register for Two Wire
	TWCR |= (1 << TWINT); //CLEAR interrupt by writing value to high


	while(!(TWCR & (1<<TWINT))); //Wait for byte to be sent before returning
		if(I2CdebugOverSerialFlag) send_serial_byte(TWSR & 0xfc); 
	TWDR = data_byte; //data register for Two Wire
	TWCR |= (1 << TWINT); //CLEAR interrupt by writing value to high

	while(!(TWCR & (1<<TWINT))); //Wait for byte to be sent before returning
		if(I2CdebugOverSerialFlag) send_serial_byte(TWSR & 0xfc);

	I2C_stop();
}

void I2C_set_start(void)
{
	TWCR |= (1 << TWSTA); //Start master transmission.;
	while(!(TWCR & (1<<TWINT)));	//Wait for start to be transmitted
}

void I2C_clear_start(void)
{
	TWCR &= ~(1 << TWSTA); //CLEAR Start master transmission
	TWCR |= (1 << TWINT); //CLEAR interrupt by writing value to high
	while(!(TWCR & (1<<TWINT)));
}

void I2C_stop(void)
{
	TWCR |= (1 << TWSTO) | (1 << TWINT); //Stop master transmission.
	while((TWCR & (1 << TWSTO))); //Wait for stop bit to be cleared. TWINT doesn't get raised for the stop command.
}

void I2C_enable_ACK(void)
{
	TWCR |= (1 << TWEA); //Enable ACK (acknowledgment).
}

void I2C_disable_ACK(void)
{
	TWCR &= ~(1 << TWEA); // disable acknowledgement
}

void readAccel(void)
{
	AxLo = I2C_read(ADXL345_DATAX0); 	
	AxHi = I2C_read(ADXL345_DATAX1);
	Ax10Bit = (signed int) (AxLo | (AxHi << 8));

	AyLo = I2C_read(ADXL345_DATAY0);
	AyHi = I2C_read(ADXL345_DATAY1);
	Ay10Bit = (signed int) (AyLo | (AyHi << 8));

	AzLo = I2C_read(ADXL345_DATAZ0);
	AzHi = I2C_read(ADXL345_DATAZ1);
	Az10Bit = (signed int) (AzLo | (AzHi << 8));
}

void initAccel(void)
{
	I2C_write(ADXL345_POWER_CTL, 0b00001000); //Put into measure mode since we power up in stand-by mode.
	I2C_write(ADXL345_DATA_FORMAT, 0b00000000); //Put into 10-bit, right-justified, +/- 2g range.
	I2C_write(ADXL345_BW_RATE, ADXL345_BW_400); //Set measurement frequency to 400 Hz.
	
}

