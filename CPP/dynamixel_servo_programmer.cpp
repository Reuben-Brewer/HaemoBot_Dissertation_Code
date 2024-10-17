#include "dynamixel_servo_programmer.h"


dynamixel_servo_programmer::dynamixel_servo_programmer(QWidget *parent, QString name_in):
    QWidget(parent)
{
    name = name_in.toStdString();
  
	/////////////////////////background_box must be the first GUI object so that it doesn't cover over other objects!!!
	background_box = new QLabel("",this);
	background_box->setGeometry(0,0,300,255);
	background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(rgb(200,255,200))}");

	connect_serial_button = new QPushButton("Connect Serial", this);
    connect_serial_button->setGeometry(QRect(5, 5, 100, 20));
    connect(connect_serial_button, SIGNAL(clicked()), this, SLOT(connect_serial()));

    reset_servo_button = new QPushButton("Reset Servo", this);
    QRect pos_reset_servo_button = QRect(5, 35, 100, 20);
    reset_servo_button->setGeometry(pos_reset_servo_button);
    connect(reset_servo_button, SIGNAL(clicked()), this, SLOT(resetServoToFactory()));

    program_servo_button = new QPushButton("Program Servo", this);
    QRect pos_program_servo_button = QRect(5, 65, 100, 20);
    program_servo_button->setGeometry(pos_program_servo_button);
    connect(program_servo_button, SIGNAL(clicked()), this, SLOT(programServo()));

    servo_target_ID_spinbox = new print_labeled_spinbox(this, "Target Servo ID", 1.0, 0.0, 0.0, 255.0, 1.0);
    QRect pos_servo_target_ID_spinbox = QRect(5, 95, 150, 30);
    servo_target_ID_spinbox->setGeometry(pos_servo_target_ID_spinbox);

    servo_new_ID_spinbox = new print_labeled_spinbox(this, "New Servo ID", 1.0, 0.0, 0.0, 255.0, 1.0);
    QRect pos_servo_new_ID_spinbox = QRect(5, 125, 150, 30);
    servo_new_ID_spinbox->setGeometry(pos_servo_new_ID_spinbox);

    servo_target_baud_spinbox = new print_labeled_spinbox(this, "Target Baud", 1.0, 0.0, 0.0, 255.0, 1.0);
    QRect pos_servo_target_baud_spinbox = QRect(5, 155, 150, 30);
    servo_target_baud_spinbox->setGeometry(pos_servo_target_baud_spinbox);

    servo_new_baud_spinbox = new print_labeled_spinbox(this, "New Baud", 1.0, 0.0, 0.0, 255.0, 1.0);
    QRect pos_servo_new_baud_spinbox = QRect(5, 185, 150, 30);
    servo_new_baud_spinbox->setGeometry(pos_servo_new_baud_spinbox);

    servo_compliance_spinbox = new print_labeled_spinbox(this, "Compliance", 1.0, 0.0, 0.0, 254.0, 1.0);
    QRect pos_servo_compliance_spinbox = QRect(5, 215, 150, 30);
    servo_compliance_spinbox->setGeometry(pos_servo_compliance_spinbox);

	comPortServoResult = -77;
	connect_serial();
}

void dynamixel_servo_programmer::timer_update(QTimerEvent *event)
{
	target_servo_ID = servo_target_ID_spinbox->getValue();
	new_servo_ID = servo_new_ID_spinbox->getValue();
	target_baud = servo_target_baud_spinbox->getValue();
	new_baud = servo_new_baud_spinbox->getValue();
	CW_compliance_margin = servo_compliance_spinbox->getValue();
	CCW_compliance_margin = CW_compliance_margin;
	CW_compliance_slope = CW_compliance_margin;
	CCW_compliance_slope = CW_compliance_margin;
}


void dynamixel_servo_programmer::send_serial_and_selfread(std::vector<unsigned int> message_vec)
{
       //HAD TO CHANGE DURING LINUX TO WINDOWS CONVERSION/// unsigned char return_buffer[message_vec.size()];
	unsigned char return_buffer[21];

	int i;
	for(i = 0; i < message_vec.size(); i++)
	{
        int temp = message_vec[i];
		comPortServo->writeChar(message_vec[i]);
		Sleep(1); //in mS NEEDED TO WORK IN WINDOWS ONLY
	}

		#ifdef unix
        usleep(7000); //Necessary to work...We set the latency_timer to 1ms, and then add on a few.
		#endif unix

        int numBytes = comPortServo->readBlock(return_buffer, message_vec.size()); //We changed readBlock to return i (num ber of bytes read) instead of len.
/*
        printf("//////////////////////////////////// \n");
        printf("%d | %d | %d | %d | %d | %d | %d | %d \n", message_vec[0], message_vec[1], message_vec[2], message_vec[3], message_vec[4], message_vec[5], message_vec[6], message_vec[7]);

        for(int j = 0; j < message_vec.size(); j++)
        {
            printf("%d ", return_buffer[j]);

        }
        printf("||| %d \n", numBytes);
        printf("//////////////////////////////////// \n");*/
}

void dynamixel_servo_programmer::resetServoToFactory()
{
	//reset 0XFF 0XFF 0X00 0X02 0X06 0XF7`
	unsigned int header = 0xFF;
	unsigned int length = 2;
	unsigned int instruction = 0x06; //RESET
	unsigned int checksum = ((target_servo_ID + length + instruction ));
	checksum = ~checksum;

	if(checksum > 255)
		checksum = checksum%256;

	unsigned int message_array[] = {header, header, target_servo_ID, length, instruction, checksum};
	std::vector<unsigned int> message_vec(message_array, message_array + sizeof(message_array)/sizeof(unsigned int));

	send_serial_and_selfread(message_vec);
}

void dynamixel_servo_programmer::programServo()
{
	setServoId();
	setServoStatusReturnLevel(1);
	setServoCompliance();
	//setServoBaud();
}

void dynamixel_servo_programmer::setServoId()
{
	//example on page 19.
	unsigned int header = 0xFF;
	unsigned int length = 4;
	unsigned int instruction = 0x03; //WRITE_DATA
	unsigned int port_register = 0x03; //ID
	unsigned int checksum = ((target_servo_ID + length + instruction + port_register + new_servo_ID));
	checksum = ~checksum;

	if(checksum > 255)
		checksum = checksum%256;

	unsigned int message_array[] = {header, header, target_servo_ID, length, instruction, port_register, new_servo_ID, checksum};
	std::vector<unsigned int> message_vec(message_array, message_array + sizeof(message_array)/sizeof(unsigned int));

	send_serial_and_selfread(message_vec);
}

void dynamixel_servo_programmer::setServoCompliance()
{
	//example on page 18.
	unsigned int header = 0xFF;
	unsigned int length = 4;
	unsigned int instruction = 0x03; //WRITE_DATA
	unsigned int port_register = 0x1A; //CW Compliance Margin
	unsigned int checksum = ((target_servo_ID + length + instruction + port_register + CW_compliance_margin + CCW_compliance_margin + CW_compliance_slope + CCW_compliance_slope));
	checksum = ~checksum;

	if(checksum > 255)
		checksum = checksum%256;

	unsigned int message_array[] = {header, header, target_servo_ID, length, instruction, port_register, CW_compliance_margin, CCW_compliance_margin, CW_compliance_slope, CCW_compliance_margin, checksum};
	std::vector<unsigned int> message_vec(message_array, message_array + sizeof(message_array)/sizeof(unsigned int));

	send_serial_and_selfread(message_vec);
}

void dynamixel_servo_programmer::setServoBaud()
{
	//table with baud rates on  on page 13.
	unsigned int header = 0xFF;
	unsigned int length = 4;
	unsigned int instruction = 0x03; //WRITE_DATA
	unsigned int port_register = 0x04; //BAUD
	unsigned int checksum = ((target_servo_ID + length + instruction + port_register + new_baud));
	checksum = ~checksum;

	if(checksum > 255)
		checksum = checksum%256;

	unsigned int message_array[] = {header, header, target_servo_ID, length, instruction, port_register, new_baud, checksum};
	std::vector<unsigned int> message_vec(message_array, message_array + sizeof(message_array)/sizeof(unsigned int));

	send_serial_and_selfread(message_vec);
}

void dynamixel_servo_programmer::setServoStatusReturnLevel(int return_level)
{
	//table on page 14.
	unsigned int header = 0xFF;
	unsigned int length = 4;
	unsigned int instruction = 0x03; //WRITE_DATA
	unsigned int port_register = 0x10; //Status Return Level
	unsigned int checksum = ((target_servo_ID + length + instruction + port_register + return_level));
	checksum = ~checksum;

	if(checksum > 255)
		checksum = checksum%256;

	unsigned int message_array[] = {header, header, target_servo_ID, length, instruction, port_register, return_level, checksum};
	std::vector<unsigned int> message_vec(message_array, message_array + sizeof(message_array)/sizeof(unsigned int));

	send_serial_and_selfread(message_vec);
}

void dynamixel_servo_programmer::move_servo(int servoID, double goal_pos, double goal_vel)
{
	unsigned char goal_pos_lo, goal_pos_hi;
	unsigned int goal_servo_pos_uint = goal_pos;
	goal_pos_lo = goal_servo_pos_uint;
	goal_pos_hi = goal_servo_pos_uint>>8;

	unsigned char goal_vel_lo, goal_vel_hi;
	unsigned int goal_servo_vel_uint = goal_vel;
	goal_vel_lo = goal_servo_vel_uint;
	goal_vel_hi = goal_servo_vel_uint>>8;

	unsigned int header = 0xFF;
	unsigned int length = 7;
	unsigned int instruction = 0x03;
	unsigned int port_register = 0x1E;
	unsigned int checksum = ((servoID + length + instruction + port_register + goal_pos_lo + goal_pos_hi + goal_vel_lo + goal_vel_hi));
	checksum = ~checksum;

	if(checksum > 255)
		checksum = checksum%256;

	unsigned int message_array[] = {header, header, servoID, length, instruction, port_register, goal_pos_lo, goal_pos_hi, goal_vel_lo, goal_vel_hi, checksum};
	std::vector<unsigned int> message_vec(message_array, message_array + sizeof(message_array)/sizeof(unsigned int));
	send_serial_and_selfread(message_vec);
}


void dynamixel_servo_programmer::toggleLED(int servoID, bool led_status)
{
	unsigned int header = 0xFF;
	unsigned int length = 4;
	unsigned int instruction = 0x03;
	unsigned int port_register = 0x19;
	unsigned int checksum = ((servoID + length + instruction + port_register + led_status));
	checksum = ~checksum;

	if(checksum > 255)
		checksum = checksum%256;

	unsigned int message_array[] = {header, header, servoID, length, instruction, port_register, led_status, checksum};
	std::vector<unsigned int> message_vec(message_array, message_array + sizeof(message_array)/sizeof(unsigned int));
	send_serial_and_selfread(message_vec);
}

double dynamixel_servo_programmer::readPosVel(int servoID)
{
	unsigned int header = 0xFF;
	unsigned int length = 4;
	unsigned int instruction = 0x02; //READ_DATA
	unsigned int port_register = 0x24; //Present Position (Lo)
	unsigned int read_length = 2; //just position bytes
	unsigned int checksum = ((servoID + length + instruction + port_register + read_length ));
	checksum = ~checksum;

	if(checksum > 255)
		checksum = checksum%256;

	unsigned int message_array[] = {header, header, servoID, length, instruction, port_register, read_length, checksum};
	std::vector<unsigned int> message_vec(message_array, message_array + sizeof(message_array)/sizeof(unsigned int));
	send_serial_and_selfread(message_vec);

	Sleep(2);

	//READ 8 BYTES BACK
	unsigned int pos_lo, pos_hi;
	unsigned int actual_servo_pos_uint = -1;
	double actual_servo_pos_double = -1;
	int return_message_length = 8;
        unsigned char return_buffer[60];
	unsigned char temp[1];
	int serial_return_flag;
        int dead_man_constant = 20;
	int TO_counter = 0;
	

        //Used to work for windows
        while(true)
	{
		return_buffer[0] = return_buffer[1];
		serial_return_flag = comPortServo->readBlock(temp,1);

		if(serial_return_flag == 1)
		{
			return_buffer[1] = temp[0];
			if(return_buffer[0] == 0xFF && return_buffer[1] == 0xFF)
				break;
		}
		TO_counter = TO_counter + 1;
		if(TO_counter == dead_man_constant)
		{
			//printf("1 to \n");
			break;
		}
	}

	TO_counter = 0;
	int counter = 2;
	while(true) 
	{
		serial_return_flag = comPortServo->readBlock(temp,1);

		if(serial_return_flag == 1)
		{
			return_buffer[counter] = temp[0];
			counter = counter + 1;
			if(counter == 8)
				break;
		}
		TO_counter = TO_counter + 1;
		if(TO_counter == dead_man_constant)
		{
			//printf("2 to \n");
			break;
		}
	}


        //////comPortServo->readBlock(return_buffer, 60);

	unsigned int calc_received_checksum = (return_buffer[2] + return_buffer[3] + return_buffer[4] + return_buffer[5] + return_buffer[6]);
	calc_received_checksum = ~calc_received_checksum;

	if(calc_received_checksum > 255)
		calc_received_checksum = calc_received_checksum%256;


	if(return_buffer[0] == 0xFF && return_buffer[1] == 0xFF && return_buffer[2] == servoID && return_buffer[3] == 4 && return_buffer[4] == 0)// && return_buffer[7] == calc_received_checksum)
	{
		pos_lo = return_buffer[5];
		pos_hi = return_buffer[6];

		actual_servo_pos_uint = (pos_lo|(pos_hi<<8));

		actual_servo_pos_double = actual_servo_pos_uint;
	}
pos_lo = return_buffer[5];
		pos_hi = return_buffer[6];

		actual_servo_pos_uint = (pos_lo|(pos_hi<<8));

		
		actual_servo_pos_double = actual_servo_pos_uint;
		//printf("%d\n", actual_servo_pos_uint);
        //printf("%d | %d | %d | %d | %d | %d | %d | %d ||| %d\n", return_buffer[0], return_buffer[1], return_buffer[2], return_buffer[3], return_buffer[4], return_buffer[5], return_buffer[6], return_buffer[7], actual_servo_pos_uint);
	

	return actual_servo_pos_double;
}

void dynamixel_servo_programmer::connect_serial(void)
{
	if(comPortServoResult != 0) //Come into if statement disconnected
	{
		comPortServo = new rlSerial;
		int COM_block = 0;
		int COM_rtscts = 0;
		comPortServoResult = comPortServo->openDevice("COM4", B1000000, COM_block, COM_rtscts, 8, 1, rlSerial::NONE);
		comPortServo->select(1); //Sets timeout. MUST HAVE THIS TO WORK IN WINDOWS
		printf("Open Dynamixel Servo COM Return Status: %d \n", comPortServoResult);

		if(comPortServoResult == 0) //if you just successfully connected.
		{
			connect_serial_button->setEnabled(0);
		}
	}
	else
	{
		cout << "Dynamixel Servo COM already open, doesn't need to be reopened" << endl;
	}
}

void dynamixel_servo_programmer::close(void)
{
    //Sleep(1000); //Wait for a second to make sure that
    comPortServo->closeDevice(); //close the serial port
}