#ifndef _PRINT_MOTOR_INFO_H
#define _PRINT_MOTOR_INFO_H 1

#include <math.h>
#include <qapplication.h>
#include <qlabel.h>
#include <qlayout.h>
#include <qwt_slider.h>
#include <qwt_scale_engine.h>
#include <qwt_scale_map.h>
#include "print_labels.h"
#include "print_labeled_spinbox.h"
#include <qwidget.h>
#include <qwt_slider.h>
#include <qlabel.h>
#include <qwt_plot.h>
#include<QRadioButton>
//#include "print_labels.h"
#include<QDoubleSpinBox>
#include <qapplication.h>
#include <qmainwindow.h>
#include <qwt_counter.h>
#include <qtoolbar.h>
#include "data_plot.h"
#include <QMutex>
#include <QSize>
#include <QThread>
#include <QWaitCondition>
#include <QApplication>
#include <QFont>
#include <QLCDNumber>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>
#include <QWidget>
#include "sliders.h"
#include <QDialog>
#include<QGroupBox>
#include<QFormLayout>
#include<QRadioButton>
#include<QSpinBox>
#include "robot_defines.h"
#include <iostream>



//class QLabel;
class QWidget;
class print_labels;
class print_labeled_spinbox;

class print_motor_info: public QWidget
{
    Q_OBJECT

public:
    print_motor_info(QWidget *parent, int motor_number_in, QString motor_name_in = NULL, double Kp_start_val_in = -1, double Kv_PIDcontroller_start_val_in = -1, double Kv_velController_start_val_in = -1, double Ki_start_val_in = -1, double vel_filter_omega_start_val_in = -1, double Kt_in = -1, double motor_V_over_A_in = -1, double voltage_for_max_cont_current_from_copely_in = -1, double calibration_speed_for_starting_ZERO_on_home_sensor_in = 0, double calibration_speed_for_starting_ONE_on_home_sensor_in = 0, int control_mode_start_in = 1, double position_offset_for_needleBot_to_assistoBot_calibration_in = 0, double percent_effort_of_max_continuous_init_in = 1.0);
	void timer_update(QTimerEvent *e);
    void enable_in_software(void);
    void disable_in_software(void);
	void setEnableStateAllSpinboxes(int val);
	void compute_errors_and_voltage(void);
	double voltage_saturation(double val_in, double max_V, double min_V);
	void update_position_and_velocity(double pos_in, double vel_in);
	void update_home_and_limit_flags(int home_flag_in, int neg_slot_flag_in, int neg_mag_flag_in, int pos_slot_flag_in, int pos_mag_flag_in);
	void setControlMode(int val);

	QLabel *background_box, *box_title, *motor_enable_hardware_flag_label, *position_label, *velocity_label, *acceleration_label, *error_label, *errorD_label, *errorSum_label, *percent_effort_of_max_continuous, *voltage_to_write_raw_label, *voltage_to_write_limited_label, *vel_filter_lambda_label, *vel_calc_deltaT_and_F_label;
	QLabel *calibration_performed_flag_label, *home_flag_label, *mag_flag_label, *slot_flag_label;
	QRadioButton *motor_enable_in_software_button, *enable_motor_constants_spinbox_change_button;
    print_labeled_spinbox *Kp_spinbox, *Kv_PIDcontroller_spinbox, *Kv_velController_spinbox, *Ki_spinbox, *max_allowable_current_percentage_spinbox, *vel_filter_omega_spinbox, *control_mode_spinbox;
	int motor_number;
	int control_mode;
	QString motor_name;

	double max_allowable_current_percentage, voltage_for_max_allowable_current;
	double position_actual, velocity_actual, acceleration_actual, position_desired, velocity_desired, acceleration_desired, error, errorD, errorSum, Kp, Kv_PIDcontroller, Kv_velController, Ki, torque_to_write, current_to_write, voltage_to_write_raw, voltage_to_write_limited, vel_filter_omega, vel_filter_lambda;
	double last_position_actual, last_velocity_actual, current_global_time, last_call_time, loop_deltaT, loop_F;
	double percent_effort_max_continous;
	int motor_enable_software_flag, last_motor_enable_software_flag;
	int motor_enable_hardware_flag, last_motor_enable_hardware_flag;
	int motor_enabled_overall_flag;
	int enable_motor_constants_spinbox_flag, last_enable_motor_constants_spinbox_flag;
	double Kp_start_val, Kv_PIDcontroller_start_val, Kv_velController_start_val, Ki_start_val, Kt, motor_V_over_A, voltage_for_max_cont_current_from_copely, vel_filter_omega_start_val;
	int calibration_performed_flag, home_flag_calibration_starting_value;
	int home_flag, neg_slot_flag, neg_mag_flag, pos_slot_flag, pos_mag_flag;
	double calibration_speed_for_starting_ZERO_on_home_sensor, calibration_speed_for_starting_ONE_on_home_sensor;
	int control_mode_start;
	double position_offset_for_needleBot_to_assistoBot_calibration;
	double percent_effort_of_max_continuous_init;

public Q_SLOTS:


protected:
    

private:
    

	
	
};

#endif
