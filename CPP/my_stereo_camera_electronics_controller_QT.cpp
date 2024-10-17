#include "my_stereo_camera_electronics_controller_QT.h"

using namespace std;

union my_union
{
    float float_num;
    char char_num[4];
};

my_stereo_camera_electronics_controller_QT::my_stereo_camera_electronics_controller_QT(QWidget *parent, QString name_in, int stereo_camera_electronics_controller_QT_width_in, int stereo_camera_electronics_controller_QT_height_in):
    QWidget(parent)
{

    stereo_camera_electronics_controller_QT_width = stereo_camera_electronics_controller_QT_width_in;
    stereo_camera_electronics_controller_QT_height = stereo_camera_electronics_controller_QT_height_in;
    print_all_bytes_flag = 0;
    name = name_in.toStdString();
    message_to_be_sent = 1;
    Tx_enabled = true;
    Rx_time_from_micro = -1; Rx_debug_from_micro = -1;


    serialRxMutex = new QMutex();


    background_box = new QLabel("",this);
    background_box->setGeometry(QRect(0, 0, stereo_camera_electronics_controller_QT_width, stereo_camera_electronics_controller_QT_height));
    background_box->setFont(QFont("Times", 12, QFont::Bold));
    background_box->setStyleSheet("*{color:red; border: 2px solid rgb(0,0,0); border-radius: 5px; background-color: rgb(200,200,255)}");

    background_text = new QLabel("Stereo Camera Electronics Controller QT",this);
    background_text->setGeometry(QRect(40, 10, 300, 30));
    background_text->setFont(QFont("Times", 12, QFont::Bold));
    background_text->setStyleSheet("*{color:black; border: 2px solid rgb(200,200,255); border-radius: 5px; background-color: rgb(200,200,255)}");

    Rx_debug_print = new QLabel("",this);
    Rx_debug_print->setGeometry(QRect(80, 40, 250, 30));
    Rx_debug_print->setFont(QFont("Times", 10));
    Rx_debug_print->setStyleSheet("*{color:black; border: 2px solid rgb(200,200,255); border-radius: 5px; background-color: rgb(200,200,255)}");


    enable_Tx_button = new QRadioButton("Tx Enable", this);
    enable_Tx_button->setGeometry(QRect(10,70,120,20));
    enable_Tx_button->setAutoExclusive(0);
    enable_Tx_button->setChecked(1);
    connect(enable_Tx_button, SIGNAL(toggled(bool)), this, SLOT(change_Tx_enabling(bool)));

    enable_mouse_control_button = new QRadioButton("Enable Mouse Control", this);
    enable_mouse_control_button->setGeometry(QRect(120,70,160,20));
    enable_mouse_control_button->setAutoExclusive(0);
    mouse_control_enabled = true;
    enable_mouse_control_button->setChecked(mouse_control_enabled);
    connect(enable_mouse_control_button, SIGNAL(toggled(bool)), this, SLOT(change_mouse_control_enabling(bool)));


    serialComPort = new rlSerial;
    int COM_block = 0;
    int COM_rtscts = 0;
    serialComPortResult = serialComPort->openDevice("COM7", B500000, COM_block, COM_rtscts, 8, 1, rlSerial::NONE); //B115200
    serialComPort->select(1); //Sets timeout. MUST HAVE THIS TO WORK IN WINDOWS
    printf("Open stereo_camera_electronics_controller_QT COM Return Status: %d \n", serialComPortResult);

    int slider_X_inc = 10;
    int slider_Y_inc = 10;
    int slider_width = 380;
    int slider_height = 60;

    int radio_button_X_start = 10;
    int radio_button_Y_start = 130;
    int radio_button_width = 100;
    int radio_button_height = 20;

    int max_FPS = 30;
    int min_FPS = 0;
    camera_FPS =   1;//initialize variable
    camera_FPS_slider = new slider_with_box(this, "Camera FPS", 0, min_FPS, max_FPS, 1, camera_FPS);
    camera_FPS_slider->setGeometry(QRect(radio_button_X_start + radio_button_width + slider_X_inc, radio_button_Y_start + 0*(slider_height + slider_Y_inc) - 35, slider_width, slider_height));
    connect(camera_FPS_slider, SIGNAL(valueChanged(double)), this, SLOT(camera_FPS_slider_changed(double)));
    is_camera_FPS_slider_dragged_flag = false;

    int laser_speed_max = 255;
    int laser_speed_min = -255;

    laser_0_speed = 0; //initialize variable
    laser_0_speed_slider = new slider_with_box(this, "Laser Speed 0", 0, laser_speed_min, laser_speed_max, 1, laser_0_speed);
    laser_0_speed_slider->setGeometry(QRect(radio_button_X_start + radio_button_width + slider_X_inc, radio_button_Y_start + 1*(slider_height + slider_Y_inc) - 35, slider_width, slider_height));
    connect(laser_0_speed_slider, SIGNAL(valueChanged(double)), this, SLOT(laser_speed_0_slider_changed(double)));
    is_laser_0_speed_slider_dragged_flag = false;

    laser_0_state = false; //initialize variable
    laser_0_state_button = new QRadioButton("Laser 0 State", this);
    laser_0_state_button->setGeometry(QRect(radio_button_X_start, radio_button_Y_start + 1*(slider_height + slider_Y_inc),radio_button_width,radio_button_height));
    laser_0_state_button->setAutoExclusive(0);
    laser_0_state_button->setChecked(laser_0_state);
    connect(laser_0_state_button, SIGNAL(toggled(bool)), this, SLOT(laser_state_0_button_changed(bool)));

    laser_1_speed = 0;//initialize variable
    laser_1_speed_slider = new slider_with_box(this, "Laser Speed 1", 0, laser_speed_min, laser_speed_max, 1, laser_1_speed);
    laser_1_speed_slider->setGeometry(QRect(radio_button_X_start + radio_button_width + slider_X_inc, radio_button_Y_start + 2*(slider_height + slider_Y_inc) - 35, slider_width, slider_height));
    connect(laser_1_speed_slider, SIGNAL(valueChanged(double)), this, SLOT(laser_speed_1_slider_changed(double)));
    is_laser_1_speed_slider_dragged_flag = false;

    laser_1_state = false; //initialize variable
    laser_1_state_button = new QRadioButton("Laser 1 State", this);
    laser_1_state_button->setGeometry(QRect(radio_button_X_start, radio_button_Y_start + 2*(slider_height + slider_Y_inc),radio_button_width,radio_button_height));
    laser_1_state_button->setAutoExclusive(0);
    laser_1_state_button->setChecked(laser_1_state);
    connect(laser_1_state_button, SIGNAL(toggled(bool)), this, SLOT(laser_state_1_button_changed(bool)));
}

void my_stereo_camera_electronics_controller_QT::timer_update(QTimerEvent *event)
{
    //serialRxMutex->lock();
        Rx_debug_print->setText("Rx Debug: Time " + QString::number(Rx_time_from_micro) + ", Debug " + QString::number(Rx_debug_from_micro));
    //serialRxMutex->unlock();
}


void my_stereo_camera_electronics_controller_QT::send_serial(std::vector<unsigned int> message_vec)
{
        for(int i = 0; i < message_vec.size(); i++)
	{
                serialComPort->writeChar(message_vec[i]);
                Sleep(1);//windows only, it's usleep in linux THIS PAUSE IS THE ONLY THING THAT MAKES IT WORK IN WINDOWS!!! //no longer needed at baud rate = 0.5MBS 11/20/2012
	}
}


void my_stereo_camera_electronics_controller_QT::TxSerialMessage(void)
{
        unsigned int message_array[] = {0xFF, 0x00, 0xFF, 0x00, 0x01}; //header plus message 1 = move steppers
        std::vector<unsigned int> message_vec(message_array, message_array + sizeof(message_array)/sizeof(unsigned int));

        unsigned int camera_FPS_uint = camera_FPS;
        message_vec.push_back(camera_FPS_uint);

        unsigned int laser_0_state_uint = laser_0_state;
        message_vec.push_back(laser_0_state_uint);

        unsigned int laser_0_speed_HIbyte_uint;
        if(laser_0_speed >= 0)
        {
            laser_0_speed_HIbyte_uint = 255;
        }
        else
        {
            laser_0_speed_HIbyte_uint = 0;
        }
        unsigned int laser_0_speed_LObyte_uint = abs(laser_0_speed);

        message_vec.push_back(laser_0_speed_LObyte_uint);
        message_vec.push_back(laser_0_speed_HIbyte_uint);

        unsigned int laser_1_state_uint = laser_1_state;
        message_vec.push_back(laser_1_state_uint);

        unsigned int laser_1_speed_HIbyte_uint;
        if(laser_1_speed >= 0)
        {
            laser_1_speed_HIbyte_uint = 255;
        }
        else
        {
            laser_1_speed_HIbyte_uint = 0;
        }
        unsigned int laser_1_speed_LObyte_uint = abs(laser_1_speed);

        message_vec.push_back(laser_1_speed_LObyte_uint);
        message_vec.push_back(laser_1_speed_HIbyte_uint);

        unsigned int checksum_uint = 77; //CHANGE!
        message_vec.push_back(checksum_uint);

        if(Tx_enabled == true)
        {
            send_serial(message_vec);
        }
}


void my_stereo_camera_electronics_controller_QT::readMessage(bool & message_done_flag)
{
        int temp;
        int received_checksum, calculated_checksum;
        static int message_number, message_length, message_counter;
        static unsigned int RxMessage[14];
        static bool message_being_processed = 0;
        my_union debug_union, time_union;

        if(message_being_processed == 0)
        {
            for(int i = 0; i <= 4; i++)
            {
                RxMessage[i] = RxMessage[i+1];
            }


            temp = serialComPort->readChar();

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
                    message_length = 13; //Counting the first byte as index 0.
                }
                else if(message_number == 2)
                {
                    message_length = 13; //Counting the first byte as index 0.
                }
            }
        }
        else if(message_being_processed == 1)
        {

            if(message_counter <= message_length)
            {
                temp = serialComPort->readChar();


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
                    //serialRxMutex->lock();
                        time_union.char_num[0] = RxMessage[5];
                        time_union.char_num[1] = RxMessage[6];
                        time_union.char_num[2] = RxMessage[7];
                        time_union.char_num[3] = RxMessage[8];
                        Rx_time_from_micro = time_union.float_num;

                        debug_union.char_num[0] = RxMessage[9];
                        debug_union.char_num[1] = RxMessage[10];
                        debug_union.char_num[2] = RxMessage[11];
                        debug_union.char_num[3] = RxMessage[12];
                        Rx_debug_from_micro = debug_union.float_num;

                        received_checksum = RxMessage[13];
                        calculated_checksum = received_checksum; //CHANGE!!!
                    //serialRxMutex->unlock();
                }
                else if(message_number == 2)
                {

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

void my_stereo_camera_electronics_controller_QT::change_Tx_enabling(bool checked)
{
    Tx_enabled = checked;
}

void my_stereo_camera_electronics_controller_QT::change_mouse_control_enabling(bool checked)
{
    mouse_control_enabled = checked;

    camera_FPS_slider->setEnabled(mouse_control_enabled);
    laser_0_speed_slider->setEnabled(mouse_control_enabled);
    laser_0_state_button->setEnabled(mouse_control_enabled);
    laser_1_speed_slider->setEnabled(mouse_control_enabled);
    laser_1_state_button->setEnabled(mouse_control_enabled);

    if(mouse_control_enabled == true)
    {
        enable_mouse_control_button->setText("Mouse Control");
    }
    else
    {
        enable_mouse_control_button->setText("Auto Control");
    }
}

void my_stereo_camera_electronics_controller_QT::laser_state_0_button_changed(bool checked)
{
    laser_0_state = checked;
}

void my_stereo_camera_electronics_controller_QT::laser_state_1_button_changed(bool checked)
{
    laser_1_state = checked;
}

void my_stereo_camera_electronics_controller_QT::camera_FPS_slider_changed(double value)
{
    camera_FPS = value;
    is_camera_FPS_slider_dragged_flag = true;
}

void my_stereo_camera_electronics_controller_QT::laser_speed_0_slider_changed(double value)
{
    laser_0_speed = value;
    is_laser_0_speed_slider_dragged_flag = true;
}

void my_stereo_camera_electronics_controller_QT::laser_speed_1_slider_changed(double value)
{
    laser_1_speed = value;
    is_laser_1_speed_slider_dragged_flag = true;
}

void my_stereo_camera_electronics_controller_QT::close()
{
    TxSerialMessage(); //Tx all zeros
    Sleep(1000); //Wait for a second to make sure that
    serialComPort->closeDevice(); //close the serial port
}
