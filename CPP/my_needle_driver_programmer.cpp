#include "my_needle_driver_programmer.h"

union my_union
{
    float float_num;
    char char_num[4];
};

my_needle_driver_programmer::my_needle_driver_programmer(QWidget *parent, QString name_in):
    QWidget(parent)
{
    print_all_bytes_flag = 0;
    name = name_in.toStdString();
    message_to_be_sent = 1;
    Tx_enabled = true;
    isCathLoopSLiderDraggedFlag = 0;
    isCathSwingArmSLiderDraggedFlag = 0;

	/////////////////////////background_box must be the first GUI object so that it doesn't cover over other objects!!!
	background_box = new QLabel("",this);
	background_box->setGeometry(0,0,400,1250);
	background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(rgb(200,255,200))}");

    NeedleDriverRxMutex = new QMutex();

    int cath_loop_slider_box_vel_START_VAL = 100;
    int cath_swing_arm_slider_box_vel_START_VAL = 150;
    int cath_loop_slider_box_vel_MAX_VAL = 255;
    int cath_swing_arm_slider_box_vel_MAX_VAL = 200;

    limitSwitches.push_back(0);
    limitSwitches.push_back(0);
    limitSwitches.push_back(0);
    limitSwitches.push_back(0);
    limitSwitches.push_back(0);
    accelData.push_back(0);
    accelData.push_back(0);
    accelData.push_back(0);
    goal_pos_vec.push_back(0);
    goal_pos_vec.push_back(0);
    goal_vel_vec.push_back(cath_loop_slider_box_vel_START_VAL);
    goal_vel_vec.push_back(cath_swing_arm_slider_box_vel_START_VAL);
    laserState = 0;

    int cath_loop_start_x = 5;
    int cath_loop_start_y = 830;

    cath_loop_slider_box_vel = new slider_with_box(this, "Cath Loop Vel", 0, 0, cath_loop_slider_box_vel_MAX_VAL, 1, cath_loop_slider_box_vel_START_VAL);
    cath_loop_slider_box_vel->setGeometry(QRect(cath_loop_start_x, cath_loop_start_y, 380, 60));
    connect(cath_loop_slider_box_vel, SIGNAL(valueChanged(double)), this, SLOT(cath_loop_vel_slider_changed()));

    cath_loop_button = new QPushButton("Open Cath Loop", this);
    cath_loop_button->setGeometry(QRect(cath_loop_start_x, cath_loop_start_y+60, 200, 20));
    connect(cath_loop_button, SIGNAL(clicked()), this, SLOT(CathLoopButtonToggle()));

    cath_loop_open_label = new QLabel("", this);
    cath_loop_open_label->setGeometry(QRect(cath_loop_start_x, cath_loop_start_y+90, 100, 20));

    cath_loop_closed_label = new QLabel("", this);
    cath_loop_closed_label->setGeometry(QRect(cath_loop_start_x+100, cath_loop_start_y+90, 100, 20));

    int cath_swing_arm_start_x = 5;
    int cath_swing_arm_start_y = 940;

    cath_swing_arm_slider_box_vel = new slider_with_box(this, "Cath Loop Vel", 0, 0, cath_swing_arm_slider_box_vel_MAX_VAL, 1, cath_swing_arm_slider_box_vel_START_VAL);
    cath_swing_arm_slider_box_vel->setGeometry(QRect(cath_swing_arm_start_x, cath_swing_arm_start_y, 380, 60));
    connect(cath_swing_arm_slider_box_vel, SIGNAL(valueChanged(double)), this, SLOT(cath_swing_arm_vel_slider_changed()));

    cath_swing_arm_button = new QPushButton("Open Cath Swing Arm", this);
    cath_swing_arm_button->setGeometry(QRect(cath_swing_arm_start_x, cath_swing_arm_start_y+60, 200, 20));
    connect(cath_swing_arm_button, SIGNAL(clicked()), this, SLOT(CathSwingArmButtonToggle()));

    cath_swing_arm_open_label = new QLabel("", this);
    cath_swing_arm_open_label->setGeometry(QRect(cath_swing_arm_start_x, cath_swing_arm_start_y+90, 100, 20));

    cath_swing_arm_closed_label = new QLabel("", this);
    cath_swing_arm_closed_label->setGeometry(QRect(cath_swing_arm_start_x+100, cath_swing_arm_start_y+90, 100, 20));

    int capacitive_indicator_box_start_x = 10;
    int capacitive_indicator_box_start_y = 630;
    capacitive_indicator_box = new QLabel("Not Inserting",this);
    capacitive_indicator_box->setGeometry(QRect(capacitive_indicator_box_start_x, capacitive_indicator_box_start_y, 100, 100));
    capacitive_indicator_box->setPixmap(QPixmap("touchState1.png").scaledToWidth(100));
    capacitive_indicator_box_text = new QLabel("Not Inserting",this);
    capacitive_indicator_box_text->setGeometry(capacitive_indicator_box_start_x,capacitive_indicator_box_start_y+20, 100, 20);
    capacitive_indicator_box_text->setFont(QFont("Times", 12, QFont::Bold));

    cath_insertion_homed_label = new QLabel("", this);
    cath_insertion_homed_label->setGeometry(QRect(10,730, 100, 20));

    enable_Tx_button = new QRadioButton("Tx Enable", this);
    QRect pos_enable_Tx_button = QRect(10,770,120,20);
    enable_Tx_button->setGeometry(pos_enable_Tx_button);
    enable_Tx_button->setAutoExclusive(0);
    enable_Tx_button->setChecked(1);
    connect(enable_Tx_button, SIGNAL(toggled(bool)), this, SLOT(change_Tx_enabling(bool)));

    laserButton = new QRadioButton("Laser State", this);
    laserButton->setGeometry(QRect(10,800,120,20));
    laserButton->setAutoExclusive(0);
    laserButton->setChecked(0);
    connect(laserButton, SIGNAL(toggled(bool)), this, SLOT(changeLaserState(bool)));

    double accel_min = -512;
    double accel_max = 512;
    double plot_height = 175;
    double plot_width = 370;

    plot_data_vec_Ax.push_back(&accelData[0]);
    plot_Ax = new DataPlot(plot_data_vec_Ax, 1, accel_min, accel_max, plot_height, "Ax", "ticks", 0, this, NeedleDriverRxMutex);
    plot_Ax->setGeometry(QRect(0, 50, plot_width, plot_height));

    plot_data_vec_Ay.push_back(&accelData[1]);
    plot_Ay = new DataPlot(plot_data_vec_Ay, 1, accel_min, accel_max, plot_height, "Ay", "ticks", 0, this, NeedleDriverRxMutex);
    plot_Ay->setGeometry(QRect(0, 250, plot_width, plot_height));

    plot_data_vec_Az.push_back(&accelData[2]);
    plot_Az = new DataPlot(plot_data_vec_Az, 1, accel_min, accel_max, plot_height, "Az", "ticks", 0, this, NeedleDriverRxMutex);
    plot_Az->setGeometry(QRect(0, 450, plot_width, plot_height));

    plot_data_vec_stringpot_voltage.push_back(&stringpot_voltage); 
	//plot_data_vec_stringpot_voltage.push_back(&stringpot_position); //////////////////////////////////////////////////USE THIS LINE TO PLOT THE POSITION AS WELL
    plot_stringpot_voltage = new DataPlot(plot_data_vec_stringpot_voltage, 1, 0.0, 5.0, plot_height, "Stringpot V", "V", 0, this, NeedleDriverRxMutex);
    plot_stringpot_voltage->setGeometry(QRect(0, 1065, plot_width, plot_height));

    comPortNeedleDriver = new rlSerial;
    int COM_block = 0;
    int COM_rtscts = 0;
    int comPortNeedleDriverResult  = comPortNeedleDriver->openDevice("COM6", B115200, COM_block, COM_rtscts, 8, 1, rlSerial::NONE);
    comPortNeedleDriver->select(1); //Sets timeout. MUST HAVE THIS TO WORK IN WINDOWS
    printf("Open Needle Driver COM Return Status: %d \n", comPortNeedleDriver);

    Rx_debug_print = new QLabel("",this);
    Rx_debug_print->setGeometry(QRect(5, 5, 390, 30));
    Rx_debug_print->setFont(QFont("Times", 12));
    Rx_debug_print->setStyleSheet("*{color:black; border: 2px solid rgb(200,200,255); border-radius: 5px; background-color: rgb(200,200,255)}");
}

void my_needle_driver_programmer::timer_update(QTimerEvent *event)
{
    Rx_debug_print->setText("Needle Driver Micro Rx Debug: " + QString::number(Rx_debug_from_micro));

    plot_Ax->update_data_plot(event); //mutex gets locked within this function
    plot_Ay->update_data_plot(event); //mutex gets locked within this function
    plot_Az->update_data_plot(event); //mutex gets locked within this function
	plot_stringpot_voltage->update_data_plot(event); //mutex gets locked within this function

    if(isCathLoopSLiderDraggedFlag == false)
    {
        cath_loop_slider_box_vel->setVal(goal_vel_vec[0]);
    }
    if(isCathLoopSLiderDraggedFlag == false)
    {
        cath_swing_arm_slider_box_vel->setVal(goal_vel_vec[1]);
    }

    if(limitSwitches[0] == 1)
    {
        cath_loop_open_label->setText("Open Switch");
    }
    else if(limitSwitches[0] == 0)
    {
        cath_loop_open_label->setText("");
    }
    if(limitSwitches[1] == 1)
    {
        cath_loop_closed_label->setText("Closed Switch");
    }
    else if(limitSwitches[1] == 0)
    {
        cath_loop_closed_label->setText("");
    }
    if(limitSwitches[2] == 1)
    {
        cath_swing_arm_open_label->setText("Open Switch");
    }
    else if(limitSwitches[2] == 0)
    {
        cath_swing_arm_open_label->setText("");
    }
    if(limitSwitches[3] == 1)
    {
        cath_swing_arm_closed_label->setText("Closed Switch");
    }
    else if(limitSwitches[3] == 0)
    {
        cath_swing_arm_closed_label->setText("");
    }
    if(touchState == 1)
    {
        capacitive_indicator_box->setHidden(false);
        capacitive_indicator_box_text->setText("Touching");
    }
    else if(touchState == 0)
    {
        capacitive_indicator_box->setHidden(true);
        capacitive_indicator_box_text->setText("Not Touching");
    }
    if(limitSwitches[4] == 1)
    {
        cath_insertion_homed_label->setText("Homed");
    }
    else if(limitSwitches[4] == 0)
    {
        cath_insertion_homed_label->setText("Not Homed");
    }
}


void my_needle_driver_programmer::send_serial(std::vector<unsigned int> message_vec)
{
        for(int i = 0; i < message_vec.size(); i++)
	{
                comPortNeedleDriver->writeChar(message_vec[i]);
                Sleep(1); //in mS
	}
}



void my_needle_driver_programmer::commandStates(void)
{
        unsigned int message_array[] = {0xFF, 0x00, 0xFF, 0x00, 0x01}; //header plu message 1 = move steppers
        std::vector<unsigned int> message_vec(message_array, message_array + sizeof(message_array)/sizeof(unsigned int));

        for(int i = 0; i < goal_pos_vec.size(); i++)
        {
            unsigned int goal_servo_pos_uint = goal_pos_vec[i];
            message_vec.push_back(goal_servo_pos_uint);

            unsigned int goal_servo_vel_uint = goal_vel_vec[i];
            unsigned int  goal_vel_lo = goal_servo_vel_uint;
            message_vec.push_back(goal_vel_lo);
        }


        message_vec.push_back(laserState);

        unsigned int checksum = 1; //CHANGE!
        message_vec.push_back(checksum);

        if(Tx_enabled == true)
        {
            send_serial(message_vec);
        }
}


void my_needle_driver_programmer::readMessage(bool & message_done_flag)
{
        int temp;
        int received_checksum, calculated_checksum;
        static int message_number, message_length, message_counter;
        static unsigned int RxMessage[26];
        static bool message_being_processed = 0;
        my_union debug_union, time_union;

        if(message_being_processed == 0)
        {
            for(int i = 0; i <= 4; i++)
            {
                RxMessage[i] = RxMessage[i+1];
            }


            temp = comPortNeedleDriver->readChar();

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
                    message_length = 25; //Counting the first byte as index 0.
                }
                else if(message_number == 2)
                {
                    message_length = 7; //Counting the first byte as index 0.
                }
            }
        }
        else if(message_being_processed == 1)
        {

            if(message_counter <= message_length)
            {
                temp = comPortNeedleDriver->readChar();


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
                    NeedleDriverRxMutex->lock();

                        /*
                        ///////////////////////////////////print all Rx data
                        for(int i = 0; i <= message_length; i++)
                        {
                            printf("i: %d  , val: %d  ", i, RxMessage[i]);
                        }
                        printf("\n");
                        ///////////////////////////////////
                        */

                        time_union.char_num[0] = RxMessage[5];
                        time_union.char_num[1] = RxMessage[6];
                        time_union.char_num[2] = RxMessage[7];
                        time_union.char_num[3] = RxMessage[8];
                        time_from_micro = time_union.float_num;


                        touchState = RxMessage[9];

                        limitSwitches[0] = RxMessage[10];
                        limitSwitches[1] = RxMessage[11];
                        limitSwitches[2] = RxMessage[12];
                        limitSwitches[3] = RxMessage[13];
                        limitSwitches[4] = RxMessage[14];

                        accelData[0] = ((signed short)(RxMessage[15] | (RxMessage[16]<<8))); //signed short because the int on this computer is 16bit
                        accelData[1] = ((signed short)(RxMessage[17] | (RxMessage[18]<<8))); //signed short because the int on this computer is 16bit
                        accelData[2] = ((signed short)(RxMessage[19] | (RxMessage[20]<<8))); //signed short because the int on this computer is 16bit


                        //printf("time: %lf char: %d char: %d\n", time_from_micro, RxMessage[17], RxMessage[18]);
                        //printf("%lf %lf %lf %lf %lf\n", time_from_micro, accelData[0], accelData[1], accelData[2], RxMessage[14]);

                        debug_union.char_num[0] = RxMessage[21];
                        debug_union.char_num[1] = RxMessage[22];
                        debug_union.char_num[2] = RxMessage[23];
                        debug_union.char_num[3] = RxMessage[24];
                        Rx_debug_from_micro = debug_union.float_num;

                        received_checksum = RxMessage[25];
                        calculated_checksum = received_checksum; //CHANGE!!!
                    NeedleDriverRxMutex->unlock();
                }
                else if(message_number == 2)
                {
                    received_checksum = RxMessage[7];
                    calculated_checksum = received_checksum; //CHANGE!!!
                }

                if(print_all_bytes_flag == 1)
                {
                    for(int j = 0; j <= message_length; j++)
                    {
                        if(j != message_length)
                         {
                            printf("%d |", RxMessage[j]);
                        }
                        else
                        {
                            printf("%d \n", RxMessage[j]);
                        }
                    }
                }

                if(received_checksum != calculated_checksum)
                {

                }

                message_done_flag = true;
                return;
            }
        }
}

void my_needle_driver_programmer::change_Tx_enabling(bool checked)
{
    Tx_enabled = checked;
}

void my_needle_driver_programmer::changeLaserState(bool checked)
{
    cout << "laser state" << checked << endl;
    laserState = checked;
}

void my_needle_driver_programmer::CathLoopButtonToggle(void)
{
   if(goal_pos_vec[0] == 0)
   {
       goal_pos_vec[0] = 1;
       cath_loop_button->setText("Close cath loop");
   }
   else if(goal_pos_vec[0] == 1)
   {
       goal_pos_vec[0] = 0;
       cath_loop_button->setText("Open cath loop");
   }
}

void my_needle_driver_programmer::setCathLoopState(bool val)
{
   goal_pos_vec[0] = val;
   if(val == 0)
   {
       cath_loop_button->setText("Open cath loop");
   }
   else if(goal_pos_vec[0] == 1)
   {
       cath_loop_button->setText("Close cath loop");
   }
}

void my_needle_driver_programmer::CathSwingArmButtonToggle(void)
{
    if(goal_pos_vec[1] == 0)
    {
        goal_pos_vec[1] = 1;
        cath_swing_arm_button->setText("Close Cath Swing Arm");
    }
    else if(goal_pos_vec[1] == 1)
    {
        goal_pos_vec[1] = 0;
        cath_swing_arm_button->setText("Open Cath Swing Arm");
    }
}

void my_needle_driver_programmer::setCathSwingArmState(bool val)
{
	goal_pos_vec[1] = val;

    if(val == 0)
    {
        cath_swing_arm_button->setText("Open Cath Swing Arm");
    }
    else if(val == 1)
    {
        cath_swing_arm_button->setText("Close Cath Swing Arm"); 
    }
}

void my_needle_driver_programmer::cath_loop_vel_slider_changed(void)
{
     goal_vel_vec[0] = cath_loop_slider_box_vel->value;
     isCathLoopSLiderDraggedFlag = true;
}

void my_needle_driver_programmer::cath_swing_arm_vel_slider_changed(void)
{
         goal_vel_vec[1] = cath_swing_arm_slider_box_vel->value;
         isCathSwingArmSLiderDraggedFlag = true;
}

void my_needle_driver_programmer::close(void)
{
    //Sleep(1000); //Wait for a second to make sure that
    comPortNeedleDriver->closeDevice(); //close the serial port
}

