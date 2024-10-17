#ifndef MY_NEEDLE_DRIVER_PROGRAMMER_H
#define MY_NEEDLE_DRIVER_PROGRAMMER_H

#include "slider_with_box.h"
#include <string>
#include <vector>
#include <QRadioButton>
#include <QPushButton>
#include "rlserial.h"
#include "data_plot.h"
#include <QMutex>
#include "print_labeled_spinbox.h"
#include <QLabel>
#include <iostream>

class my_needle_driver_programmer: public QWidget
{
    Q_OBJECT
public:
    my_needle_driver_programmer(QWidget *parent, QString name_in = "");
    void timer_update(QTimerEvent *e);
    void readMessage(bool & message_done_flag);
    void commandStates(void);
    void send_serial(std::vector<unsigned int> message_vec);
	void setCathLoopState(bool val);
	void setCathSwingArmState(bool val);
	void close(void);

    std::string name;
    int stepper_ID, com_open_result;

    std::vector<double> goal_pos_vec;
    std::vector<double> goal_vel_vec;
    std::vector<unsigned char> limitSwitches;
    std::vector<double> accelData;
    bool touchState;
    double time_from_micro;
    bool laserState;

    bool Tx_enabled;

    QPushButton *cath_loop_button, *cath_swing_arm_button;
    QLabel *cath_loop_open_label, *cath_loop_closed_label, *cath_swing_arm_open_label, *cath_swing_arm_closed_label, *capacitive_indicator_box, *capacitive_indicator_box_text, *cath_insertion_homed_label;
    QRadioButton *enable_Tx_button, *laserButton;
    rlSerial *comPortNeedleDriver;
    int message_to_be_sent;
    float Rx_debug_from_micro;
    QMutex *NeedleDriverRxMutex;
    DataPlot *plot_Ax, *plot_Ay, *plot_Az, *plot_stringpot_voltage;
	double stringpot_voltage, stringpot_position;
    std::vector<double *> plot_data_vec_Ax, plot_data_vec_Ay, plot_data_vec_Az, plot_data_vec_stringpot_voltage;
    bool print_all_bytes_flag;
    slider_with_box *cath_loop_slider_box_vel, *cath_swing_arm_slider_box_vel;
    bool isCathLoopSLiderDraggedFlag, isCathSwingArmSLiderDraggedFlag;
    QLabel *Rx_debug_print,*background_box;


private Q_SLOTS:
    void change_Tx_enabling(bool checked);
    void changeLaserState(bool checked);
    void CathLoopButtonToggle(void);
    void CathSwingArmButtonToggle(void);
    void cath_loop_vel_slider_changed(void);
    void cath_swing_arm_vel_slider_changed(void);
};



#endif
