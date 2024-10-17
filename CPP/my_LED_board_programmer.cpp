#include "my_LED_board_programmer.h"

using namespace std;

union my_union
{
    float float_num;
    char char_num[4];
};

my_LED_board_programmer::my_LED_board_programmer(QWidget *parent, QString name_in, int LED_board_programmer_width_in, int LED_board_programmer_height_in):
    QWidget(parent)
{

    LED_board_programmer_width = LED_board_programmer_width_in;
    LED_board_programmer_height = LED_board_programmer_height_in;
    print_all_bytes_flag = 0;
    name = name_in.toStdString();
    message_to_be_sent = 1;
    Tx_enabled = true;
    Rx_time_from_micro = -1; Rx_debug_from_micro = -1;


    LED_board_RxMutex = new QMutex();


    background_box = new QLabel("",this);
    background_box->setGeometry(QRect(0, 0, LED_board_programmer_width, LED_board_programmer_height));
    background_box->setFont(QFont("Times", 12, QFont::Bold));
    background_box->setStyleSheet("*{color:red; border: 2px solid rgb(0,0,0); border-radius: 5px; background-color: rgb(200,200,255)}");

    background_text = new QLabel("LED Board Programmer",this);
    background_text->setGeometry(QRect(100, 10, 200, 30));
    background_text->setFont(QFont("Times", 12, QFont::Bold));
    background_text->setStyleSheet("*{color:black; border: 2px solid rgb(200,200,255); border-radius: 5px; background-color: rgb(200,200,255)}");

    Rx_debug_print = new QLabel("",this);
    Rx_debug_print->setGeometry(QRect(10, 40, 320, 30));
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

    enable_mouse_control_all_button = new QRadioButton("Single Mouse", this);
    enable_mouse_control_all_button->setGeometry(QRect(275,70,160,20));
    enable_mouse_control_all_button->setAutoExclusive(0);
    mouse_control_all = false;
    enable_mouse_control_all_button->setChecked(mouse_control_all);
    connect(enable_mouse_control_all_button, SIGNAL(toggled(bool)), this, SLOT(change_mouse_single_or_all(bool)));



    comPortLEDboard = new rlSerial;
    int COM_block = 0;
    int COM_rtscts = 0;
    comPortLEDboardResult = comPortLEDboard->openDevice("COM8", B500000, COM_block, COM_rtscts, 8, 1, rlSerial::NONE); ///dev/myttyLEDboard used to be B115200
    comPortLEDboard->select(1); //Sets timeout. MUST HAVE THIS TO WORK IN WINDOWS
    printf("Open LED Board COM Return Status: %d \n", comPortLEDboardResult);

    int test_LED_brightness = 50;

    signal_mapper = new QSignalMapper(this);
    connect(signal_mapper, SIGNAL(mapped(int)),this, SLOT(select_LED(int)));

    int x_counter = 0;
    int y_counter = 0;
    LED_width = 50;
    LED_height = 50;
    int x_start = 80;
    int y_start = 155;
    for(int N_counter = 0; N_counter <= 19; N_counter++)
    {
        y_counter = N_counter/4; //because y_counter is an int, the result disgards the decimal portion, leaving just the number of whole divisions
        x_counter = N_counter - 4*y_counter;
        LED_button_vec.push_back(new QPushButton("", this));
        LED_button_vec[N_counter]->setFont(QFont("Times", 12, QFont::Bold));
        LED_button_vec[N_counter]->setGeometry(QRect(x_start + x_counter*LED_width, y_start + y_counter*LED_height, LED_width, LED_height));
        LED_button_vec[N_counter]->setStyleSheet(generateLEDstyleSheet(0,0,0,0));
        LED_button_vec[N_counter]->setText(QString::number(N_counter) + ", " + QString::number(0));

        signal_mapper->setMapping(LED_button_vec[N_counter], N_counter);
        connect(LED_button_vec[N_counter], SIGNAL(clicked()), signal_mapper, SLOT(map()));
        LED_brightness_vec.push_back(0);
        LED_selected_vec.push_back(0);
    }

    brightness_slider = new slider_with_box(this, "Brightness", 0, 0, 255, 1, 0);
    brightness_slider->setGeometry(QRect(10, 85, 380, 60));
    connect(brightness_slider, SIGNAL(valueChanged(double)), this, SLOT(brightness_slider_changed()));




}

void my_LED_board_programmer::timer_update(QTimerEvent *event)
{
    //LED_board_RxMutex->lock();
        debug_from_computer = LED_brightness_vec[19];
        Rx_debug_print->setText("Rx Debug: Time " + QString::number(Rx_time_from_micro) + ", Debug Micro " + QString::number(Rx_debug_from_micro) + ", Debug Comp " + QString::number(debug_from_computer));
    //LED_board_RxMutex->unlock();
}


void my_LED_board_programmer::send_serial(std::vector<unsigned int> message_vec)
{
        for(int i = 0; i < message_vec.size(); i++)
	{
                comPortLEDboard->writeChar(message_vec[i]);
                //Sleep(1);//windows only, it's usleep in linux THIS PAUSE IS THE ONLY THING THAT MAKES IT WORK IN WINDOWS!!! //no longer needed at baud rate = 0.5MBS 11/20/2012
	}
}



void my_LED_board_programmer::TxAllBrightnessLevels(void)
{
        unsigned int message_array[] = {0xFF, 0x00, 0xFF, 0x00, 0x01}; //header plus message 1 = move steppers
        std::vector<unsigned int> message_vec(message_array, message_array + sizeof(message_array)/sizeof(unsigned int));

        for(int i = 0; i < LED_brightness_vec.size(); i++)
        {
            unsigned int LED_brightness_uint = LED_brightness_vec[i];
            message_vec.push_back(LED_brightness_uint);
        }

        unsigned int checksum = 77; //CHANGE!
        message_vec.push_back(checksum);

        if(Tx_enabled == true)
        {
            send_serial(message_vec);
        }
}


void my_LED_board_programmer::readMessage(bool & message_done_flag)
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


            temp = comPortLEDboard->readChar();

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
                temp = comPortLEDboard->readChar();


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
                    //LED_board_RxMutex->lock();
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
                    //LED_board_RxMutex->unlock();
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

void my_LED_board_programmer::change_Tx_enabling(bool checked)
{
    Tx_enabled = checked;
}

void my_LED_board_programmer::change_mouse_control_enabling(bool checked)
{
    mouse_control_enabled = checked;

    change_button_enabling(mouse_control_enabled);
    brightness_slider->setEnabled(mouse_control_enabled);
    enable_mouse_control_all_button->setEnabled(mouse_control_enabled);

    if(mouse_control_enabled == true)
    {
        enable_mouse_control_button->setText("Mouse Control");
    }
    else
    {
        enable_mouse_control_button->setText("Auto Control");
    }
}


void my_LED_board_programmer::change_button_enabling(bool checked)
{
    for(int i = 0; i < LED_brightness_vec.size(); i++)
    {
        LED_button_vec[i]->setEnabled(checked);
    }
}


void my_LED_board_programmer::change_mouse_single_or_all(bool checked)
{
    mouse_control_all = checked;
    if(mouse_control_all == true)
    {
        enable_mouse_control_all_button->setText("Multi Mouse");
        for(int i = 0; i < LED_brightness_vec.size(); i++)
        {
            LED_selected_vec[i] = true;
            updateLED(i, 0);
            brightness_slider->setVal(0);
        }
        LED_num_selected = LED_brightness_vec.size();
        change_button_enabling(0);
    }
    else
    {
        enable_mouse_control_all_button->setText("Single Mouse");
        for(int i = 0; i < LED_brightness_vec.size(); i++)
        {
            LED_selected_vec[i] = false;
            updateLED(i, LED_brightness_vec[i]);
        }
        LED_num_selected = 0;
        change_button_enabling(1);
    }
}

void my_LED_board_programmer::select_LED(int button_num)
{
    if(LED_selected_vec[button_num] == true)
    {
        LED_selected_vec[button_num] = false; //deselect
        LED_num_selected -= 1;
    }
    else
    {
        LED_selected_vec[button_num] = true; //select
        LED_num_selected += 1;
    }

    if(LED_num_selected > 1)
    {
        for(int i = 0; i < LED_brightness_vec.size(); i++)
        {
            if(LED_selected_vec[i] == true)
            {
                updateLED(i, 0);
            }
        }

    }

    brightness_slider->setVal(LED_brightness_vec[button_num]);

    updateLED(button_num, LED_brightness_vec[button_num]);
}

QString my_LED_board_programmer::generateLEDstyleSheet(int borderR,int borderG,int borderB, int background_gray_value)
{
    QString rgb_border_color_string = "rgb(" + QString::number(borderR) + "," + QString::number(borderG) + "," + QString::number(borderB) + ")";
    QString rgb_background_color_string = "rgb(" + QString::number(background_gray_value) + "," + QString::number(background_gray_value) + "," + QString::number(background_gray_value) + ")";
    QString style_sheet_string = "*{color:red; border: 2px solid " + rgb_border_color_string + "; border-radius: 0px; background-color: " + rgb_background_color_string + "}"; //color:red sets the text/font color
    return style_sheet_string;
}

void my_LED_board_programmer::brightness_slider_changed(void)
{

    for(int i = 0; i < LED_brightness_vec.size(); i++)
    {
        if(LED_selected_vec[i] == true)
        {
            updateLED(i, brightness_slider->value);
        }
    }

    isBrightnessSliderDraggedFlag = true;
}

void my_LED_board_programmer::updateLED(int LEDnum, int brightness_value)
{
    LED_brightness_vec[LEDnum] = brightness_value; //Update the brightness level
    LED_button_vec[LEDnum]->setText(QString::number(LEDnum) + ", " + QString::number(brightness_value)); //Update the displayed text
    if(LED_selected_vec[LEDnum] == true)
    {
        LED_button_vec[LEDnum]->setStyleSheet(generateLEDstyleSheet(255,0,0,brightness_value)); //Update the displayed background color, border red
    }
    else
    {
        LED_button_vec[LEDnum]->setStyleSheet(generateLEDstyleSheet(brightness_value,brightness_value,brightness_value,brightness_value)); //Update the displayed background color, border matches background
    }
}


void my_LED_board_programmer::close()
{
    for(int i = 0; i < LED_brightness_vec.size(); i++)
    {
       updateLED(i, 0);
    }

    TxAllBrightnessLevels(); //Tx all zeros
    Sleep(1000); //Wait for a second to make sure that
    comPortLEDboard->closeDevice(); //close the serial port
}
