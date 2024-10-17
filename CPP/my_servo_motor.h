#ifndef MY_SERVO_H
#define MY_SERVO_H

#include "slider_with_box.h"
#include <string>
#include <vector>
#include <QRadioButton>
#include <QPushButton>
#include "data_plot.h"
#include <QMutex>
#include "print_labeled_spinbox.h"
#include "phidget_servo_motor.h"
#include <vector>

class my_servo: public QWidget
{
    Q_OBJECT
public:
    my_servo(QWidget *parent, vector<double> servo_start_angle_in);
    void timer_update(QTimerEvent *e);
    void enable(int num);
    void open(void);
    void close(void);
    void disable(int num);
    void move_motor(int num, double pos);
    bool isEnabled(int num);
    void updatePosition(int num);

    phidget_servo_motor *my_servo_motor;
    std::vector<slider_with_box *> slider_box_pos;
    std::vector<slider_with_box *> slider_box_vel;
    std::vector<slider_with_box *> slider_box_accel;
    std::vector<DataPlot *> plot;
    std::string motor_name;
    int motor_ID;
    std::vector<QRadioButton *> enable_button;

    int num_servos;
    std::vector<double> actual_pos, goal_pos, actual_vel, goal_accel, actual_accel, goal_vel;
    float vel_min, vel_max, accel_min, accel_max;

    bool isDraggedFlag;
    std::vector<bool> enabled_flag;
    QMutex *motor_mutex;



private Q_SLOTS:
        void change_enabling(bool checked);
        void pos_slider_changed(void);
        void vel_or_accel_slider_changed(void);
};



#endif
