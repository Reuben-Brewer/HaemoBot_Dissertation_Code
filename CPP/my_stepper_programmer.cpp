#include "my_stepper_programmer.h"

union my_union
{
    float float_num;
    char char_num[4];
};

my_stepper_programmer::my_stepper_programmer(QWidget *parent, QString name_in):
    QWidget(parent)
{
    name = name_in.toStdString();
  
	PosDesired = std::vector<double>(3,0);
	Pos_omniInput_raw = std::vector<double>(3,0); 
	Pos_omniInput_rezero_snapshot = std::vector<double>(3,0);
	Pos_omniInput_offset = std::vector<double>(3,0);
	Pos_manualSliderInput = std::vector<double>(3,0);

	/////////////////////////background_box must be the first GUI object so that it doesn't cover over other objects!!!
	background_box = new QLabel("",this);
	background_box->setGeometry(0,0,510,340);
	background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(rgb(200,255,200))}");

    home_all_steppers_button = new QPushButton("Home All Steppers", this);
    QRect pos_home_all_steppers_button = QRect(5, 5, 100, 20);
    home_all_steppers_button->setGeometry(pos_home_all_steppers_button);
    connect(home_all_steppers_button, SIGNAL(clicked()), this, SLOT(home_all_steppers()));

	rezero_omni_button = new QPushButton("Rezero Omni", this);
    rezero_omni_button->setGeometry(QRect(110, 5, 100, 20));
    connect(rezero_omni_button, SIGNAL(clicked()), this, SLOT(take_rezero_omni_snapshot()));

	connect_serial_button = new QPushButton("Connect Serial", this);
    connect_serial_button->setGeometry(QRect(215, 5, 100, 20));
    connect(connect_serial_button, SIGNAL(clicked()), this, SLOT(connect_serial()));

	Tx_enabled = true;
    enable_Tx_button = new QRadioButton("Tx Enable", this);
    QRect pos_enable_Tx_button = QRect(5,35,120,20);
    enable_Tx_button->setGeometry(pos_enable_Tx_button);
    enable_Tx_button->setAutoExclusive(0);
    enable_Tx_button->setChecked(Tx_enabled);
    connect(enable_Tx_button, SIGNAL(toggled(bool)), this, SLOT(change_Tx_enabling(bool)));

	manual_control_flag = true;
	last_manual_control_flag = true;
    manual_control_button = new QRadioButton("Manual Control", this);
    manual_control_button->setGeometry(QRect(5,65,120,20));
    manual_control_button->setAutoExclusive(0);
    manual_control_button->setChecked(manual_control_flag);
    //connect(manual_control_button, SIGNAL(toggled(bool)), this, SLOT(change_manual_control(bool)));

	joint_coords_flag = false;
    joint_coords_button = new QRadioButton("Joint Coords", this);
    joint_coords_button->setGeometry(QRect(5,85,120,20));
    joint_coords_button->setAutoExclusive(0);
    joint_coords_button->setChecked(joint_coords_flag);
    //connect(joint_coords_button, SIGNAL(toggled(bool)), this, SLOT(change_joint_coords(bool)));

    Xmin = -58.5;
    Xmax = 58.5;
    Xinc = 1.0;
    Xinit = -50.0;
	PosDesired[0] = Xinit;
	Pos_manualSliderInput[0] = Xinit;

    Ymin = -50.0;
    Ymax = 58.5;
    Yinc = 1.0;
    Yinit = -40.0;
	PosDesired[1] = Yinit;
	Pos_manualSliderInput[1] = Yinit;

    Zmin = 0.0;
    Zmax = 94.0;
    Zinc = 1.0;
    Zinit = 94.0;
	PosDesired[2] = Zinit;
	Pos_manualSliderInput[2] = Zinit;

	int labels_start_width = 150;
	int labels_start_height = 20;

    slider_box_manual_x = new slider_with_box(this, "manual X", 0, Xmin, Xmax, Xinc, Xinit);
    slider_box_manual_x->setGeometry(QRect(5, 105, 380, 60));
    connect(slider_box_manual_x, SIGNAL(valueChanged(double)), this, SLOT(x_slider_changed()));

	X_desired_label = new QLabel("", this);
    X_desired_label->setGeometry(QRect(390, 105, labels_start_width, labels_start_height));

    slider_box_manual_y = new slider_with_box(this, "manual Y", 0, Ymin, Ymax, Yinc, Yinit);
    slider_box_manual_y->setGeometry(QRect(5, 185, 380, 60));
    connect(slider_box_manual_y, SIGNAL(valueChanged(double)), this, SLOT(y_slider_changed()));

	Y_desired_label = new QLabel("", this);
    Y_desired_label->setGeometry(QRect(390, 185, labels_start_width, labels_start_height));

    slider_box_manual_z = new slider_with_box(this, "manual Z", 0, Zmin, Zmax, Zinc, Zinit);
    slider_box_manual_z->setGeometry(QRect(5, 255, 380, 60));
    connect(slider_box_manual_z, SIGNAL(valueChanged(double)), this, SLOT(z_slider_changed()));

	Z_desired_label = new QLabel("", this);
    Z_desired_label->setGeometry(QRect(390, 255, labels_start_width, labels_start_height));

	comPortStepperResult = -77;
	connect_serial();

    message_to_be_sent = 2; //default to homing message

    isDraggedFlag = false;
}

void my_stepper_programmer::timer_update(QTimerEvent *event)
{
	///////////////////////////////////////////////////////////////////////////////////  In case we use the foot pedal to change the manual/haptic control flags, we need to be able to change the radio buttons but from within the GUI thread.
	if(manual_control_button_needs_to_be_checked == 1)
	{
		manual_control_button->setChecked(manual_control_flag);
		manual_control_button_needs_to_be_checked = 0;
	}
	///////////////////////////////////////////////////////////////////////////////////

	manual_control_flag = manual_control_button->isChecked();
	joint_coords_flag = joint_coords_button->isChecked();

	if(manual_control_flag == 0 && last_manual_control_flag == 1)
	{
		take_rezero_omni_snapshot(); //WHEN WE FIRST START HAPTIC CONTROL, MAKE SURE THAT WE'RE PROPERLY ZEROED FIRST!!!
	}

	X_desired_label->setText("Xdesired: " + QString::number(PosDesired[0]));
	Y_desired_label->setText("Ydesired: " + QString::number(PosDesired[1]));
	Z_desired_label->setText("Zdesired: " + QString::number(PosDesired[2]));

	last_manual_control_flag = manual_control_flag;
}


void my_stepper_programmer::send_serial(std::vector<unsigned int> message_vec)
{
        for(int i = 0; i < message_vec.size(); i++)
	{
                comPortStepper->writeChar(message_vec[i]);
                Sleep(1); //in mS, must have for this to work in Windows
	}
}



void my_stepper_programmer::move_stepper(std::vector<double> goal_pos_vec, std::vector<double> goal_vel_vec, std::vector<bool> move_mode, double BP_goal_pressure)
{
        unsigned int message_array[] = {0xFF, 0x00, 0xFF, 0x00, 0x01}; //header plu message 1 = move steppers
        std::vector<unsigned int> message_vec(message_array, message_array + sizeof(message_array)/sizeof(unsigned int));

        for(int i = 0; i < goal_pos_vec.size(); i++)
        {
            message_vec.push_back(move_mode[i]);

            unsigned int goal_servo_pos_uint = goal_pos_vec[i];
            unsigned int goal_pos_lo = goal_servo_pos_uint;
            unsigned int goal_pos_hi = goal_servo_pos_uint>>8;
            message_vec.push_back(goal_pos_lo);
            message_vec.push_back(goal_pos_hi);

            unsigned int goal_servo_vel_uint = goal_vel_vec[i];
            unsigned int  goal_vel_lo = goal_servo_vel_uint;
            unsigned int goal_vel_hi = goal_servo_vel_uint>>8;
            message_vec.push_back(goal_vel_lo);
            message_vec.push_back(goal_vel_hi);
        }

        unsigned int BP_goal_pressure_uint = BP_goal_pressure;
        unsigned int BP_goal_pressure_uint_lo = BP_goal_pressure_uint;
        unsigned int BP_goal_pressure_uint_hi = BP_goal_pressure_uint>>8;

        message_vec.push_back(BP_goal_pressure_uint_lo);
        message_vec.push_back(BP_goal_pressure_uint_hi);

        unsigned int checksum = 1; //CHANGE!
        message_vec.push_back(checksum);

        if(Tx_enabled == true)
        {
            send_serial(message_vec);
        }
}


void my_stepper_programmer::home_steppers(std::vector<bool> motors_home, std::vector<unsigned char> motors_home_speed)
{
        unsigned int message_array[] = {0xFF, 0x00, 0xFF, 0x00, 0x02}; //header plus message 2 = home stepper
        std::vector<unsigned int> message_vec(message_array, message_array + sizeof(message_array)/sizeof(unsigned int));

        for(int i = 0; i < motors_home.size(); i++)
        {
            message_vec.push_back(motors_home[i]);
        }

        for(int i = 0; i < motors_home_speed.size(); i++)
        {
            message_vec.push_back(motors_home_speed[i]);
        }

        unsigned int checksum = 1; //CHANGE!
        message_vec.push_back(checksum);

		for(int num_times_to_send_message = 0; num_times_to_send_message <= 10; num_times_to_send_message++)
		{
			send_serial(message_vec);
		}

		cout << "my_stepper_programmer::home_steppers issued." << endl;
}


void my_stepper_programmer::readMessage(std::vector<signed char> & stepper_homed_state, std::vector<double> & stepper_actual_pos, double & BP_analog, bool & message_done_flag)
{
        int temp;
        int received_checksum, calculated_checksum;
        static int message_number, message_length, message_counter;
        static unsigned int RxMessage[30];
        static bool message_being_processed = 0;
        my_union debug_union;

        if(message_being_processed == 0)
        {
            for(int i = 0; i <= 4; i++)
            {
                RxMessage[i] = RxMessage[i+1];
            }


            temp = comPortStepper->readChar();
            if(temp  == -1)
            {
                return;
            }
            RxMessage[4] = temp;

            if(RxMessage[0] == 0xFF && RxMessage[1] == 0x00 && RxMessage[2] == 0xFF & RxMessage[3] == 0x00)
            {
                message_being_processed = 1;
                message_counter = 5;
                message_number = RxMessage[4];
                if(message_number == 1)
                {
                    message_length = 20; //Counting the first byte as index 0.
                }
                else if(message_number == 2)
                {
                    message_length = 7; //Counting the first byte as index 0.
                }

				//cout << "Received stepper programmer RX message header" << endl;
            }
        }
        else if(message_being_processed == 1)
        {

            if(message_counter <= message_length)
            {
                temp = comPortStepper->readChar();


                if(temp  == -1)
                {
                    return;
                }
                RxMessage[message_counter] = temp;
                message_counter = message_counter + 1;
            }
            if(message_counter > message_length)
            {
                message_being_processed = 0;

                if(message_number == 1)
                {
                    stepper_homed_state[0] = RxMessage[5];
                    stepper_actual_pos[0] = ((signed short)(RxMessage[6] | (RxMessage[7]<<8))); //signed short because the int on this computer is 16bit
                    stepper_homed_state[1] = RxMessage[8];
                    stepper_actual_pos[1] = ((signed short)(RxMessage[9] | (RxMessage[10]<<8)));
                    stepper_homed_state[2] = RxMessage[11];
                    stepper_actual_pos[2] = ((signed short)(RxMessage[12] | (RxMessage[13]<<8)));
                    BP_analog = ((double)(RxMessage[14] | (RxMessage[15]<<8)));
                    debug_union.char_num[0] = RxMessage[16];
                    debug_union.char_num[1] = RxMessage[17];
                    debug_union.char_num[2] = RxMessage[18];
                    debug_union.char_num[3] = RxMessage[19];
                    debug_from_micro = debug_union.float_num;
                    received_checksum = RxMessage[20];
                    calculated_checksum = received_checksum; //CHANGE!!!
					//cout << "received stepper message" << endl;

                }
                else if(message_number == 2)
                {
                    received_checksum = RxMessage[7];
                    calculated_checksum = received_checksum; //CHANGE!!!
                }


                /*for(int j = 0; j <= message_length; j++)
                {
                    if(j != message_length)
                     {
                        printf("%d |", RxMessage[j]);
                    }
                    else
                    {
                        printf("%d \n", RxMessage[j]);
                    }
                }*/

                if(received_checksum != calculated_checksum)
                {
                    stepper_actual_pos[0] = -1;
                    stepper_actual_pos[1] = -1;
                    stepper_actual_pos[2] = -1;
                }

                message_done_flag = true;
                return;
            }
        }
}

void my_stepper_programmer::home_all_steppers(void)
{
    printf("Home All Steppers! \n");

    message_to_be_sent = 2;
}

void my_stepper_programmer::change_Tx_enabling(bool checked)
{
    Tx_enabled = checked;
}

void my_stepper_programmer::change_manual_control(bool checked)
{
    manual_control_flag = checked;
    //printf("MANUAL manual_control_flag %d \n", manual_control_flag);
}

bool my_stepper_programmer::isManualControl(void)
{
    return manual_control_flag;
}

void my_stepper_programmer::x_slider_changed()
{
    Pos_manualSliderInput[0] = slider_box_manual_x->getSliderVal();
    isDraggedFlag = true;
}

void my_stepper_programmer::y_slider_changed()
{
    Pos_manualSliderInput[1] = slider_box_manual_y->getSliderVal();
    isDraggedFlag = true;
}

void my_stepper_programmer::z_slider_changed()
{
    Pos_manualSliderInput[2] = slider_box_manual_z->getSliderVal();
    isDraggedFlag = true;
}

void my_stepper_programmer::change_joint_coords(bool checked)
{
    joint_coords_flag = checked;
}

bool my_stepper_programmer::isJointCoords(void)
{
    return joint_coords_flag;
}

void my_stepper_programmer::updateOmniInput(std::vector<double> omni_pos_vec_in)
{
	Pos_omniInput_raw = omni_pos_vec_in;
	
	double temp_Pos_omniInput_offset[3];
	for(int i = 0; i < Pos_omniInput_raw.size(); i++)
	{
		temp_Pos_omniInput_offset[i] = Pos_omniInput_raw[i] - Pos_omniInput_rezero_snapshot[i];
	}

	Pos_omniInput_offset[0] = saturateValue(temp_Pos_omniInput_offset[0], Xmax, Xmin);
	Pos_omniInput_offset[1] = saturateValue(temp_Pos_omniInput_offset[1], Ymax, Ymin);
	Pos_omniInput_offset[2] = saturateValue(temp_Pos_omniInput_offset[2], Zmax, Zmin);
}

void my_stepper_programmer::take_rezero_omni_snapshot(void)
{
	Pos_omniInput_rezero_snapshot[0] = Pos_omniInput_raw[0] - PosActual[0]; //JUST ADDED THIS SO THAT WE START OFF HAPTIC CONTROL WHERE WE CURRENTLY ARE.
	Pos_omniInput_rezero_snapshot[1] = Pos_omniInput_raw[1] - PosActual[1]; //JUST ADDED THIS SO THAT WE START OFF HAPTIC CONTROL WHERE WE CURRENTLY ARE. 
	Pos_omniInput_rezero_snapshot[2] = Pos_omniInput_raw[2] - PosActual[2]; //JUST ADDED THIS SO THAT WE START OFF HAPTIC CONTROL WHERE WE CURRENTLY ARE.
	
	cout << "Took another snapshot to rezero the assistoBot Omni." << endl;
}

void my_stepper_programmer::updateActualPos(std::vector<double> PosActual_in)
{
	PosActual = PosActual_in;
}

void my_stepper_programmer::updateDesiredPosVelAccel(void)
{
	if(manual_control_flag == 1)
	{
		for(int i = 0; i < PosDesired.size(); i++)
		{
			PosDesired[i] = Pos_manualSliderInput[i];
		}
	}
	else if(manual_control_flag == 0)
	{
		for(int i = 0; i < PosDesired.size(); i++)
		{
			PosDesired[i] = Pos_omniInput_raw[i] - Pos_omniInput_rezero_snapshot[i];
		}
	}
}

void my_stepper_programmer::connect_serial(void)
{
	if(comPortStepperResult != 0) //Come into if statement disconnected
	{
		comPortStepper = new rlSerial;
		int COM_block = 0;
		int COM_rtscts = 0;
		comPortStepperResult = comPortStepper->openDevice("COM5", B115200, COM_block, COM_rtscts, 8, 1, rlSerial::NONE);
		comPortStepper->select(1); //Sets timeout. MUST HAVE THIS TO WORK IN WINDOWS
		printf("Open Stepper COM Return Status: %d \n", comPortStepperResult);

		if(comPortStepperResult == 0) //if you just successfully connected.
		{
			connect_serial_button->setEnabled(0);
		}
	}
	else
	{
		cout << "Stepper COM already open, doesn't need to be reopened" << endl;
	}
}

double my_stepper_programmer::saturateValue(double val_in, double max, double min)
{
	double val_saturated = val_in;

	if(val_in > max)
	{
		val_saturated = max;
	}
	else if(val_in < min)
	{
		val_saturated = min;
	}
	
	return val_saturated;
}

void my_stepper_programmer::set_manual_control_flag(bool val)
{
	manual_control_flag = val;
	manual_control_button_needs_to_be_checked = 1;

	if(manual_control_flag == 0 && last_manual_control_flag == 1)
	{
		take_rezero_omni_snapshot();
	}

	//last_manual_control_flag == !val;
}

void my_stepper_programmer::close(void)
{
    //Sleep(1000); //Wait for a second to make sure that
    comPortStepper->closeDevice(); //close the serial port
}


