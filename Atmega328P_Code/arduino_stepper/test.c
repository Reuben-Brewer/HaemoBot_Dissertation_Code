#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/signal.h> 
#include "SystemStart.h"

void SerialInit(void);
void ADCStart(unsigned char);
void ADCInit(void);
void PWM_TimerOneInitOC(void);
void StepTimerThreeInitOC(void);
void TxTimerTwoInitOC(void);
void externalInterruptInit(void);

volatile float last_Rx_time = 0.0;
volatile float main_loop_global_time = 0.0;
volatile float debug_to_computer;
volatile signed int BP_error;
volatile unsigned int BP_goal_pressure;
volatile unsigned int BP_analog = 0;
volatile unsigned int counterRx = 0;
volatile unsigned int message_counter = 0;
volatile unsigned int message_length = 0;
volatile unsigned int RxMessage[30];
volatile unsigned int received_checksum, calculated_checksum;
volatile int message_being_processed = 0;
volatile unsigned char motor_mode[3] = {0, 0, 0};
volatile unsigned char motor_num_to_be_homed = 0;
volatile signed int motor_goal_pos[3] = {0, 0, 0};
volatile unsigned int motor_goal_vel[3] = {0, 0, 0};
volatile signed int motor_actual_pos[3] = {0, 0, 0};
volatile unsigned int is_motor_stepping[3] = {0, 0, 0};
volatile unsigned int motor_pulse_counter[3] = {0, 0, 0};
volatile signed char motor_homing_state[3] = {-1, -1, -1}; //-1 means no homed, 0 means homing, 1 means homed
volatile unsigned char motor_toBeHomed[3] = {0, 0, 0}; //0 means shouldn't be home, 1 means should be homed
volatile unsigned int bp_pump_PWM;
volatile signed int motor_home_pos_light[3] = {32767, -1700, -1700}; //Change values!
volatile signed int motor_home_pos_dark[3] = {-32767, 1700, 1700}; //Change values! 1700 gives 180deg of motion to try
volatile signed int motor_home_speed[3] = {30, 30, 30}; //Change values!
volatile signed int motor_home_mode[3] = {0, 0, 0}; //Change values!

#define M0_DIR_PORT PORTA
#define M0_STEP_PORT PORTA
#define M0_MS1_PORT PORTA
#define M0_HOME_PORT PIND //PIND instead of PORTD because we're reading, not writing
#define M1_DIR_PORT PORTA
#define M1_STEP_PORT PORTA
#define M1_MS1_PORT PORTA
#define M1_HOME_PORT PIND //PIND instead of PORTD because we're reading, not writing
#define M2_DIR_PORT PORTA
#define M2_STEP_PORT PORTA
#define M2_MS1_PORT PORTC
#define M2_HOME_PORT PIND //PIND instead of PORTD because we're reading, not writing
#define debug_led_port PORTC
#define BP_SOLENOID_PORT PORTC


#define M0_DIR_LINE  0b00000001
#define M0_STEP_LINE 0b00000010
#define M0_MS1_LINE  0b00000100
#define M0_HOME_LINE 0b00000001
#define M1_DIR_LINE  0b00001000
#define M1_STEP_LINE 0b00010000
#define M1_MS1_LINE  0b00100000
#define M1_HOME_LINE 0b00000010
#define M2_DIR_LINE  0b01000000
#define M2_STEP_LINE 0b10000000
#define M2_MS1_LINE  0b00000001
#define M2_HOME_LINE 0b00000100
#define debug_led_line 0b00000010
#define BP_SOLENOID_LINE 0b00000100

#define dead_man_time 0.50

typedef union //c-syntax
{
	float float_num;
	char char_num[4];
} my_union; //c-syntax

inline int check_deadman_expired(void)
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

inline void ADCReadAndRespond(unsigned char Channel)
{	

	//Put the A to D converted value into an integer (it is currently split in two registers)
	BP_analog = (ADCL |(ADCH<<8));
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

inline void set_bp_solenoid_line(unsigned char val)
{
	switch(val)
	{
		case 0:
		{
			BP_SOLENOID_PORT &= ~BP_SOLENOID_LINE;
			break;
		}
		case 1:
		{
			BP_SOLENOID_PORT |= BP_SOLENOID_LINE;
			break;
		}
	}
}


inline void set_mot_dir_line(unsigned char motor_num, unsigned char val)
{
	switch(motor_num)
	{
		case 0:
		{
			if(val == 0)
			{
				M0_DIR_PORT &= ~M0_DIR_LINE; 
			}
			else if(val == 1)
			{
			 	M0_DIR_PORT |= M0_DIR_LINE;
			}
			break;
		}
		case 1:
		{
			if(val == 0)
			{
				M1_DIR_PORT &= ~M1_DIR_LINE; 
			}
			else if(val == 1)
			{
			 	M1_DIR_PORT |= M1_DIR_LINE;
			}
			break;
		}
		case 2:
		{
			if(val == 0)
			{
				M2_DIR_PORT &= ~M2_DIR_LINE; 
			}
			else if(val == 1)
			{
			 	M2_DIR_PORT |= M2_DIR_LINE;
			}
			break;
		}
	}
}

inline void set_mot_step_line(unsigned char motor_num, unsigned char val)
{
	switch(motor_num)
	{
		case 0:
		{
			if(val == 0)
			{
				M0_STEP_PORT &= ~M0_STEP_LINE; 
			}
			else if(val == 1)
			{
			 	M0_STEP_PORT |= M0_STEP_LINE;
			}
			break;
		}
		case 1:
		{
			if(val == 0)
			{
				M1_STEP_PORT &= ~M1_STEP_LINE; 
			}
			else if(val == 1)
			{
			 	M1_STEP_PORT |= M1_STEP_LINE;
			}
			break;
		}
		case 2:
		{
			if(val == 0)
			{
				M2_STEP_PORT &= ~M2_STEP_LINE; 
			}
			else if(val == 1)
			{
			 	M2_STEP_PORT |= M2_STEP_LINE;
			}
			break;
		}
	}
}


inline void set_mot_ms1_line(unsigned char motor_num, unsigned char val)
{
	switch(motor_num)
	{
		case 0:
		{
			if(val == 0)
			{
				M0_MS1_PORT &= ~M0_MS1_LINE; 
			}
			else if(val == 1)
			{
			 	M0_MS1_PORT |= M0_MS1_LINE;
			}
			break;
		}
		case 1:
		{
			if(val == 0)
			{
				M1_MS1_PORT &= ~M1_MS1_LINE; 
			}
			else if(val == 1)
			{
			 	M1_MS1_PORT |= M1_MS1_LINE;
			}
			break;
		}
		case 2:
		{
			if(val == 0)
			{
				M2_MS1_PORT &= ~M2_MS1_LINE; 
			}
			else if(val == 1)
			{
			 	M2_MS1_PORT |= M2_MS1_LINE;
			}
			break;
		}
	}
}



inline unsigned int get_mot_home_line(unsigned char motor_num)
{
	switch(motor_num)
	{
		case 0:
		{
			if((M0_HOME_PORT & M0_HOME_LINE) != 0)
			{
				return 1;
			}
			else
			{
				return 0;
			}
			break;
		}
		case 1:
		{
			if((M1_HOME_PORT & M1_HOME_LINE) != 0)
			{
				return 1;
			}
			else
			{
				return 0;
			}
			break;
		}
		case 2:
		{
			if((M2_HOME_PORT & M2_HOME_LINE) != 0)
			{
				return 1;
			}
			else
			{
				return 0;
			}
			break;
		}
	}
}


ISR(INT0_vect)
{
	if(motor_toBeHomed[0] == 1) //If we're supposed to be homing
	{
		EIMSK &= ~(1 << INT0); //Disable interrupt INT0.
		debug_led_port ^= debug_led_line; //Toggle line for debugging sensor

		motor_homing_state[0] = 1; //Say we're homed
		motor_toBeHomed[0] = 0; //We're homed, so stop homing.
		motor_goal_pos[0] = 0;
		motor_actual_pos[0] = 0; //re-zero actual_position
	}

	EIFR |= (1 << INT0); //Manually clear INT0 flag.
}

ISR(INT1_vect)
{
	if(motor_toBeHomed[1] == 1) //If we're supposed to be homing
	{
		EIMSK &= ~(1 << INT1); //Disable interrupt INT1.
		debug_led_port ^= debug_led_line; //Toggle line for debugging sensor

		motor_homing_state[1] = 1; //Say we're homed
		motor_toBeHomed[1] = 0; //We're homed, so stop homing.
		motor_goal_pos[1] = 0;
		motor_actual_pos[1] = 0; //re-zero actual_position
	}

	EIFR |= (1 << INT1); //Manually clear INT1 flag.
}

ISR(INT2_vect)
{
	if(motor_toBeHomed[2] == 1) //If we're supposed to be homing
	{
		EIMSK &= ~(1 << INT2); //Disable interrupt INT2.
		debug_led_port ^= debug_led_line; //Toggle line for debugging sensor
		
		motor_homing_state[2] = 1; //Say we're homed
		motor_toBeHomed[2] = 0; //We're homed, so stop homing.
		motor_goal_pos[2] = 0;
		motor_actual_pos[2] = 0; //re-zero actual_position
	}

	EIFR |= (1 << INT2); //Manually clear INT2 flag.
}


ISR(USART0_RX_vect)
{
	PORTB ^= 0x80; //Toggle internal board LED to show we received a serial command.

	if(message_being_processed == 0)
	{
		//Shift old data down by one slot.
		for(int i = 0; i <= 3; i++)
		{
			RxMessage[i] = RxMessage[i+1];
		}

		RxMessage[4] = UDR0; //Grab new serial data.

		//If we've received a proper header, then set the flag so that we start processing new message.
		if(RxMessage[0] == 0xFF  && RxMessage[1] == 0x00  && RxMessage[2] == 0xFF  && RxMessage[3] == 0x00)
		{
			message_being_processed = 1;
			message_counter = 5;

			if(RxMessage[4] == 1)
			{
				message_length = 21;  //Message length counting message bytes from 0.
			}
			else if(RxMessage[4] == 2)
			{
				message_length = 11; //Message length counting message bytes from 0.
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
				//FF--00--FF--00--MESSAGE_NUM(1)--MO_MODE--M0_POS_LO--M0_POS_HI--M0_VEL_LO--M0_VEL_HI--M1_MODE--M1_POS_LO--M1_POS_HI--M1_VEL_LO--M1_VEL_HI--M2_MODE--M2_POS_LO--M2_POS_HI--M2_VEL_LO--M2_VEL_HI--CHECKSUM
				//0   1   2   3   4               5        6          7          8          9          10       11         12         13         14         15       16         17         18         19         20

				

				if(motor_toBeHomed[0] == 0)
				{
					motor_mode[0] = RxMessage[5];
					motor_goal_pos[0] = (signed int) (RxMessage[6] | (RxMessage[7]<<8));
					motor_goal_vel[0] = RxMessage[8] | (RxMessage[9]<<8);
					//EIMSK |= (1 << INT0); //Enable interrupt INT0.
				}

				if(motor_toBeHomed[1] == 0)
				{
					motor_mode[1] = RxMessage[10];						
					motor_goal_pos[1] = (signed int) (RxMessage[11] | (RxMessage[12]<<8));
					motor_goal_vel[1] = RxMessage[13] | (RxMessage[14]<<8);
					//EIMSK |= (1 << INT1); //Enable interrupt INT1.
				}

				if(motor_toBeHomed[2] == 0)
				{
					motor_mode[2] = RxMessage[15];
					motor_goal_pos[2] = (signed int) (RxMessage[16] | (RxMessage[17]<<8));
					motor_goal_vel[2] = RxMessage[18] | (RxMessage[19]<<8);
					//EIMSK |= (1 << INT2); //Enable interrupt INT2.
				}

				BP_goal_pressure = RxMessage[20] | (RxMessage[21]<<8);
				received_checksum = RxMessage[22];
			}
			else if(RxMessage[4] == 2) //Motor home message
			{
				
				
				//FF--00--FF--00--MESSAGE_NUM(2)--MOTOR_0_TO_BE_HOMED--MOTOR_1_TO_BE_HOMED--MOTOR_2_TO_BE_HOMED--CHECKSUM
				//0   1   2   3   4               5                      6                  7                    8

				motor_toBeHomed[0] = RxMessage[5];
				motor_toBeHomed[1] = RxMessage[6];
				motor_toBeHomed[2] = RxMessage[7];
				motor_home_speed[0] = RxMessage[8];
				motor_home_speed[1] = RxMessage[9];
				motor_home_speed[2] = RxMessage[10];
				received_checksum = RxMessage[11];	

				for(int y = 0; y <= 2; y++)
				{
					if(motor_toBeHomed[y] == 1)
					{
						motor_actual_pos[y] = 0; //EXPERIMENTAL LINE 8/2/2011
						motor_homing_state[y] = 0;
						motor_mode[y] = motor_home_mode[y];
						motor_goal_vel[y] = motor_home_speed[y];
						if(get_mot_home_line(y) == 1) 
						{
							motor_goal_pos[y] = motor_home_pos_light[y];	
						}
						else
						{
							motor_goal_pos[y] = motor_home_pos_dark[y];
						}

						if(y == 0)
						{
							EIMSK |= (1 << INT0); //Enable interrupt INT0.
							EIFR |= (1 << INT0); //Manually clear INT0 flag.
						}
						else if(y == 1)
						{
							EIMSK |= (1 << INT1); //Enable interrupt INT1.
							EIFR |= (1 << INT1); //Manually clear INT1 flag.
						}
						else if(y == 2)
						{
							EIMSK |= (1 << INT2); //Enable interrupt INT2.
							EIFR |= (1 << INT2); //Manually clear INT2 flag.
						}

					}
				}
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


ISR(TIMER3_COMPA_vect) //start the step
{
	if(check_deadman_expired() == 0) //dead-man hasn't been tripped
	{
		for(int i = 0; i <= 2; i++)
		{
			motor_pulse_counter[i] = motor_pulse_counter[i] + 1; //increment counter
			set_mot_ms1_line(i, motor_mode[i]); //Set motor mode

			if(motor_pulse_counter[i] >= motor_goal_vel[i]) //If we've waited long enough to step...
			{
				motor_pulse_counter[i] = 0; //Clear right away so that we keep our timing correct

				if(motor_actual_pos[i] > motor_goal_pos[i])
				{
					set_mot_dir_line(i, 0); //Set motor dir
					motor_actual_pos[i] = motor_actual_pos[i] - 1;
					is_motor_stepping[i] = 1;
					set_mot_step_line(i, 1);
				}
				else if(motor_actual_pos[i] < motor_goal_pos[i])
				{
					set_mot_dir_line(i, 1); //Set motor dir
					motor_actual_pos[i] = motor_actual_pos[i] + 1;
					is_motor_stepping[i] = 1;
					set_mot_step_line(i, 1);
				}

			}
		}
		
		TIFR3 |= (1<<OCF3A); //Clear interrupt flag
	}
}


ISR(TIMER3_COMPB_vect) //end the step
{
	if(check_deadman_expired() == 0) //dead-man hasn't been tripped
	{
		TCNT3 = 0; //RESET timer 

		for(int i = 0; i <= 2; i++)
		{	
			if(is_motor_stepping[i] == 1)
			{
				set_mot_step_line(i, 0);
				is_motor_stepping[i] = 0;
			}
		}


	
		TIFR3 |= (1<<OCF3B); //Clear interrupt flag
	}
}

int main(int argc, char **argv)
{
	//All PORTA pins set as output. 
	//A0 (pin ) = Stepper0 Dir, A1 (pin ) = Stepper0 Step, A2 (pin ) = Stepper0 MS1
	//A3 (pin ) = Stepper1 Dir, A4 (pin ) = Stepper1 Step, A5 (pin ) = Stepper1 MS1
	//A6 (pin ) = Stepper2 Dir, A7 (pin ) = Stepper2 Step.

	DDRA = 0xFF;
	PORTA = 0x00; //Set all of PORTA to 0 for stepper motors.
	


	//PORTC pin set as output. C0 = Stepper2 MS1

	DDRC &= 0x01;
	PORTC &= (0<<0); //Set PC0 to 0 for stepper motor.

	PORTC &= (0<<1); //Set PC1 to 0 for debug led.
	PORTC &= (0<<2); //Set PC2 to 0 for BP solenoid.


	DDRB |= 0x80; //Set internal LED pin hi as output. 

	

	DDRF &= (0 << 2);

	SerialInit();
	ADCInit();
	PWM_TimerOneInitOC();
	TxTimerTwoInitOC();
	StepTimerThreeInitOC();
	externalInterruptInit();
	sei();

	
	
	int TxCounter = 0;	
	while(1)
	{
	
		if((TIFR2 & 0b00000010) != 0)
		{
			main_loop_global_time = main_loop_global_time + 0.001; //+ 1 mS
			


			if ((ADCSRA & 0b00010000) != 0)
			{
		
				ADCReadAndRespond(2);	
				ADCStart(2);

				BP_error = (BP_goal_pressure - BP_analog);
				bp_pump_PWM = 348;

				if(check_deadman_expired() == 0) //dead-man hasn't been tripped
				{
					
					if(BP_error > 0)
					{
						set_bp_solenoid_line(1); //Solenoid on so we don't bleed air.

						bp_pump_PWM = 696;// ((float) BP_analog/1024)*1600;

				
					}
					else if(BP_error < -10)
					{
						set_bp_solenoid_line(0); //Bleed air solenoid
					}
				}

				OCR1A = bp_pump_PWM;
			}

		



			TxCounter = TxCounter + 1;

			if(TxCounter == 10) //every 10ms
			{
			//debug_led_port ^= debug_led_line;
				send_serial_byte(0xFF); //byte 0
				send_serial_byte(0x00); //byte 1
				send_serial_byte(0xFF); //byte 2
				send_serial_byte(0x00); //byte 3
				send_serial_byte(0x01); //byte 4
				send_serial_byte((unsigned char) motor_homing_state[0]); //byte 5
				send_serial_int(motor_actual_pos[0]); //bytes 6:7 //motor_actual_pos
				send_serial_byte((unsigned char) motor_homing_state[1]); //byte 8
				send_serial_int(motor_actual_pos[1]); //byte 9:10 //motor_actual_pos
				send_serial_byte((unsigned char) motor_homing_state[2]); //byte 11
				send_serial_int(motor_actual_pos[2]); //bytes 12:13 //motor_actual_pos
				send_serial_int(BP_analog); //bytes 14:15 BP_analog

				float temp = main_loop_global_time - last_Rx_time;

				debug_to_computer = temp;
				send_serial_float(debug_to_computer); //bytes 16:19 debug message

				send_serial_byte(77); //byte 20    //CHECKSUM!!!!!!!!!

				TxCounter = 0;
			}





			TIFR2 |= (1<<OCF2A); //clear interrupt flag
		}

		


	}
	return 0;
}


void SerialInit(void)
{
	UCSR0A = _BV(U2X0);
	UCSR0B = _BV(RXCIE0) | _BV(TXEN0) | _BV(RXEN0);
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8n1
	UBRR0H = 0;
	UBRR0L = 16; // 115k2
}

//This function sets the proper ADC channel to pay attention to then starts the conversion
void ADCStart(unsigned char Channel)
{
	ADMUX = 0b00000010; //Page 291
	ADCSRA |= (1 << ADIF); //Clear the A to D conversion complete flag (done by setting it to 1)
	ADCSRA |= (1 << ADSC); //Initialize the next A to D conversion
}

void ADCInit(void)
{

	ADCSRA |= (1 << ADPS2) | (1 << ADPS1)|(1 << ADPS0); //Sets the AD prescale to 1/128 or ~125kHz

	ADCSRA |= (1 << ADEN); //ADEN enables ADC

//	ADCSRA |= (1 << ADATE); //Sets an auto trigger mode, which is then defined with ADCSRB register ADTS bits
//	ADCSRA |= (1 << ADIE);				//Enable the interrupt at each ADC conversion

    ADCStart(2);

	DDRF &= 0b00000100; //
}


void PWM_TimerOneInitOC(void)
{
	//PAGE 148, 160
	TCCR1A |= (1 << WGM11);  //Set for 16-bit fast PWM, MODE 14 (TOP = ICR0).
	TCCR1A &= ~(1 << WGM10); //Set for 16-bit fast PWM, MODE 14 (TOP = ICR0).
	
	
	//TCCR1A |= (1 << WGM10); //MODE 15
	//TCCR1A &= ~(1 << COM1A1);//For pin OC1A to be toggled on output compare. MODE 15
	//TCCR1A |= (1 << COM1A0);//For pin OC1A to be toggled on output compare. MODE 15
	
	TCCR1A |= (1 << COM1A1);//For pin OC1A to be cleared on compare match, set at bottom. MODE 14
	TCCR1A &= ~(1 << COM1A0);//For pin OC1A to be cleared on compare match, set at bottom. MODE 14

	TCCR1B |= (1 << WGM13) | (1 << WGM12); //Set for 16-bit fast PWM, MODE 14 (TOP = ICR1).

	TCCR1B |= (1 << CS10); // 1/1
	TCCR1B &= ~(1 << CS11);// 1/1
	TCCR1B &= ~(1 << CS12); // 1/1

	//ICR1 = 1600; //1600 Sets waveform frequency to 10kHz.
	ICR1 = 696; //696 Sets waveform frequency to 23kHz.

	DDRB |= (1 << PB5); //Set OC1A as an output. Pin PWM 11.
}

void TxTimerTwoInitOC(void)
{
	TCCR2A |= (1<<WGM21); //clear counter on interrupt

	TCCR2B |= (1<<CS20); //Prescaler set to 1/1024th
	TCCR2B |= (1<<CS21); //Prescaler set to 1/1024th
	TCCR2B |= (1<<CS22); //Prescaler set to 1/1024th

	//TIMSK2 |= (1<<OCIE2A); //Enable interrupt when timer 2 matches output compare.
	
	OCR2A = 16; //Outputcompare flags after 2 uS (mathematically should be 32, but minus one for logic)
}

void StepTimerThreeInitOC(void)
{
	//TCCR3B |= (1<<WGM32); //clear counter on interrupt

	TCCR3B |= (1 << CS30); // 1/64
	TCCR3B |= (1 << CS31);// 1/64
	TCCR3B &= ~(1 << CS32); // 1/64

	TIMSK3 |= (1<<OCIE3A); //ENABLE OUTPUT COMPARE INTERRUPT
	TIMSK3 |= (1<<OCIE3B); //ENABLE OUTPUT COMPARE INTERRUPT

	OCR3A = 65; // 16Set output compare //75
	OCR3B = 130; // 31Set output compare //150
}

void externalInterruptInit(void)
{
	//External Pin Interrupts detailed on page 115, 83.

	EICRA |= (1 << ISC00) | (1 << ISC10) | (1 << ISC20); //Trigger on toggle.
	DDRD &= ~0b00000111;
}




















/*
ISR(TIMER2_COMPA_vect)
{
	TIFR2 |= (1<<OCF2A); //clear interrupt flag
}
*/
/*PCICR |= (1 << PCIE0); //enable pin change interrupt 0
	PCMSK0 |= (PCINT6); //Enable PCINT6
	DDRB &= ~0b01000000;//Enable PB6 as input line.



	PCICR |= (1 << PCIE1); //enable pin change interrupt 1
	PCMSK1 |= (PCINT9); //Enable PCINT9 
	DDRJ &= ~0b00000001;//Enable UX3 as input line.

	PCICR |= (1 << PCIE2); //enable pin change interrupt 2
	PCMSK2 |= (1 << PCINT16); //Enable PCINT16line (Arduino Mega line ADC8) to be external interrupt.
	DDRK &= ~0b00000001; //Set ADC8 as input line.
	

	EIMSK |= (1 << INT4); //Enable INT4.
	EICRB |= (1 << ISC40); //Trigger on toggle.
	DDRE &= ~(1 << PE4);
	*/
