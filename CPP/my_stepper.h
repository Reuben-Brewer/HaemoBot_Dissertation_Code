#ifndef MY_STEPPER_H
#define MY_STEPPER_H

#include "slider_with_box.h"
#include <string>
#include <vector>
#include <QRadioButton>
#include <QPushButton>
#include "rlserial.h"
#include "data_plot.h"
#include <QMutex>
#include "my_stepper_programmer.h"
#include "print_labeled_spinbox.h"
#include <QLabel>

class my_stepper: public QWidget
{
    Q_OBJECT
public:
    my_stepper(QWidget *parent, QString stepper_name_in = "Stepper", int stepper_ID_in = 0, float pos_min_in = 0.0, float pos_max_in = 0.0, float pos_inc_in = 0.0, float pos_init_in = 0.0, float vel_min_in = 0.0, float vel_max_in = 0.0, float vel_inc_in = 0.0, float vel_init_in = 0.0, float home_speed_min_in = 0.0, float home_speed_max_in = 0.0, float home_speed_inc_in = 0.0, float home_speed_init_in = 0.0, float preset_angle_1_in = 0.0, float preset_angle_2_in = 0.0, my_stepper_programmer *stepper_programmer_in = NULL);
    void timer_update(QTimerEvent *e);
    void enable(void);
    void disable(void);
    void forced_update(void);
    void move_stepper(double goal_pos, double goal_vel);
    bool isEnabled(void);

    print_labeled_spinbox *stepper_ID_spinbox;
    slider_with_box *slider_box_pos, *slider_box_vel, *slider_box_home_speed;
    DataPlot *plot;
    std::string stepper_name;
    int com_open_result, stepper_ID;
    QRadioButton *enable_button;
    QPushButton *goToButton, *step_mode_button, *homeButton;
    unsigned int goal_stepper_pos_uint, goal_stepper_vel_uint;
    unsigned int actual_stepper_pos_uint, actual_stepper_vel_uint;
    double goal_stepper_pos_double, goal_stepper_vel_double, actual_stepper_pos_double, actual_stepper_vel_double, goal_full_steps, actual_full_steps, home_speed;
    float preset_angle_1, preset_angle_2, pos_min, pos_max;
    std::vector<double *> plot_data_vec;
    my_stepper_programmer *stepper_programmer;
    bool isDraggedFlag, enabled_flag, step_mode;
    QMutex *stepper_mutex;
    signed char homed_state;
    bool toBeHomed;
	QLabel *background_box;

private Q_SLOTS:
        void change_enabling(bool checked);
        void pos_or_vel_slider_changed(void);
        void goToPreset(void);
        void change_stepper_ID(double new_servo_ID);
        void change_step_mode(void);
        void home(void);
};



#endif
