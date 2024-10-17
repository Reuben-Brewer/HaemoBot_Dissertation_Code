#ifndef MY_STEREO_CAMERA_ELECTRONICS_CONTOLLER_QT
#define MY_STEREO_CAMERA_ELECTRONICS_CONTOLLER_QT

#include "slider_with_box.h"
#include <string>
#include <vector>
#include <QRadioButton>
#include <QPushButton>
#include "rlserial.h"
#include "data_plot.h"
#include <QMutex>
#include "print_labeled_spinbox.h"
#include "rlserial.h"
#include <QLabel>
#include <iostream>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

class my_stereo_camera_electronics_controller_QT: public QWidget
{
    Q_OBJECT
public:
    my_stereo_camera_electronics_controller_QT(QWidget *parent, QString name_in = "", int LED_board_programmer_width_in = 0, int LED_board_programmer_height_in = 0);
    void timer_update(QTimerEvent *e);
    void readMessage(bool & message_done_flag);
    void TxSerialMessage(void);
    void send_serial(std::vector<unsigned int> message_vec);
    void close();

    std::string name;
    int serialComPortResult;


    int stereo_camera_electronics_controller_QT_width, stereo_camera_electronics_controller_QT_height;
    QLabel *background_box, *background_text, *Rx_debug_print;

    double time_from_micro;
    bool Tx_enabled, mouse_control_enabled;

    slider_with_box *laser_0_speed_slider, *laser_1_speed_slider, *camera_FPS_slider;
    QRadioButton *enable_Tx_button, *enable_mouse_control_button, *laser_0_state_button, *laser_1_state_button;
    rlSerial *serialComPort;
    int message_to_be_sent;
    float Rx_time_from_micro, Rx_debug_from_micro;
    QMutex *serialRxMutex;
    bool print_all_bytes_flag, is_camera_FPS_slider_dragged_flag, is_laser_0_speed_slider_dragged_flag, is_laser_1_speed_slider_dragged_flag;

    int camera_FPS, laser_0_speed, laser_1_speed;
    bool laser_0_state, laser_1_state;


private Q_SLOTS:
    void change_Tx_enabling(bool checked);
    void change_mouse_control_enabling(bool checked);
    void laser_state_0_button_changed(bool checked);
    void laser_state_1_button_changed(bool checked);

    void camera_FPS_slider_changed(double value);
    void laser_speed_0_slider_changed(double value);
    void laser_speed_1_slider_changed(double value);

};



#endif
