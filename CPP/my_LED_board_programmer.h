#ifndef MY_LED_BOARD_PROGRAMMER_H
#define MY_LED_BOARD_PROGRAMMER_H

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
#include "data_plot.h"
#include <QLabel>
#include <iostream>
#include <QSignalMapper>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

class my_LED_board_programmer: public QWidget
{
    Q_OBJECT
public:
    my_LED_board_programmer(QWidget *parent, QString name_in = "", int LED_board_programmer_width_in = 0, int LED_board_programmer_height_in = 0);
    void timer_update(QTimerEvent *e);
    void readMessage(bool & message_done_flag);
    void TxAllBrightnessLevels(void);
    void send_serial(std::vector<unsigned int> message_vec);
    QString generateLEDstyleSheet(int borderR,int borderG,int borderB, int background_gray_value);
    void updateLED(int LEDnum, int brightness_value);
    void change_button_enabling(bool checked);
    void close();

    std::string name;
    int comPortLEDboardResult;

    int LED_width, LED_height, LED_num_selected;

    std::vector<double> LED_brightness_vec;
    std::vector<double> is_LED_selected_vec;
    std::vector<QPushButton *> LED_button_vec;
    std::vector<bool> LED_selected_vec;

    int LED_board_programmer_width, LED_board_programmer_height;
    QLabel *background_box, *background_text, *Rx_debug_print;
    QSignalMapper *signal_mapper;

    double time_from_micro;
    bool Tx_enabled, mouse_control_enabled, mouse_control_all;
    int LED_currently_selected, LED_last_selected;

    slider_with_box *brightness_slider;
    QRadioButton *enable_Tx_button, *enable_mouse_control_button, *enable_mouse_control_all_button;
    rlSerial *comPortLEDboard;
    int message_to_be_sent;
    float Rx_time_from_micro, Rx_debug_from_micro, debug_from_computer;
    QMutex *LED_board_RxMutex;
    bool print_all_bytes_flag, isBrightnessSliderDraggedFlag;


private Q_SLOTS:
    void change_Tx_enabling(bool checked);
    void select_LED(int button_num);
    void brightness_slider_changed(void);
    void change_mouse_control_enabling(bool checked);
    void change_mouse_single_or_all(bool checked);
};



#endif
