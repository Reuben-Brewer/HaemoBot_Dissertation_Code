#ifndef MY_DC_MOTOR_H
#define MY_DC_MOTOR_H

#include "slider_with_box.h"
#include <string>
#include <vector>
#include <QRadioButton>
#include <QPushButton>
#include "data_plot.h"
#include <QMutex>
#include "print_labeled_spinbox.h"
#include "phidget_DC_motor.h"

class my_DC_motor: public QWidget
{
    Q_OBJECT
public:
    my_DC_motor(QWidget *parent, QString stepper_name_in = "DC Motor", int motor_ID_in = 0, float CPR_in = 360.0, float kp_min_in = 0.0, float kp_max_in = 0.0, float kp_inc_in = 0.0, float kp_init_in = 0.3, float kv_min_in = 0.0, float kv_max_in = 0.0, float kv_inc_in = 0.0, float kv_init_in = 0.0, float ki_min_in = 0.0, float ki_max_in = 0.0, float ki_inc_in = 0.0, float ki_init_in = 0.0, float pos_min_in = 0.0, float pos_max_in = 0.0, float pos_inc_in = 0.0, float pos_init_in = 0.0, float vel_min_in = 0.0, float vel_max_in = 0.0, float vel_inc_in = 0.0, float vel_init_in = 0.0, float accel_min_in = 0.0, float accel_max_in = 0.0, float accel_inc_in = 0.0, float accel_init_in = 0.0);
    void timer_update(QTimerEvent *e);
    void enable(void);
    void open(double CPR);
    void close(void);
    void disable(void);
    void move_motor(void);
    bool isEnabled(void);
    void updatePosition(void);

    phidget_DC_motor *my_DC_phidget_motor;
    slider_with_box *slider_box_pos, *slider_box_vel, *slider_box_accel, *slider_box_kp, *slider_box_kv, *slider_box_ki;
    DataPlot *plot;
    std::string motor_name;
    int motor_ID;
    QRadioButton *enable_button, *vel_control_button;
    QPushButton *homeButton;

    double CPR, kp, kv, ki;
    double actual_pos, goal_pos;
    double actual_vel, goal_accel;
    double actual_accel, goal_vel;
    float vel_min, vel_max, accel_min, accel_max;

    std::vector<double *> plot_data_vec;
    bool isDraggedFlag, enabled_flag;
    QMutex *motor_mutex;
    signed char homed_state;
    bool toBeHomed;
    bool vel_control_mode;

private Q_SLOTS:
        void change_enabling(bool checked);
        void change_vel_control_mode(bool checked);
        void slider_changed(void);
        void home(void);
};



#endif
