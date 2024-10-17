#ifndef DYNAMIXEL_SERVO_H
#define DYNAMIXEL_SERVO_H

#include "slider_with_box.h"
#include <string>
#include <vector>
#include <QRadioButton>
#include <QPushButton>
#include "rlserial.h"
#include "data_plot.h"
#include <QMutex>
#include "dynamixel_servo_programmer.h"
#include "print_labeled_spinbox.h"
#include <QLabel>

class dynamixel_servo: public QWidget
{
    Q_OBJECT
public:
    dynamixel_servo(QWidget *parent, QString servo_name_in = "Servo", int servo_ID_in = 0, float pos_min_in = 0.0, float pos_max_in = 0.0, float pos_inc_in = 0.0, float pos_init_in = 0.0, float vel_min_in = 0.0, float vel_max_in = 0.0, float vel_inc_in = 0.0, float vel_init_in = 0.0, float preset_angle_1_in = 0.0, float preset_angle_2_in = 0.0, dynamixel_servo_programmer *servo_programmer_in = NULL);
    void timer_update(QTimerEvent *e);
    void enable(void);
    void disable(void);
    void forced_update(void);
    void move_servo(double goal_pos, double goal_vel);
    bool isEnabled(void);
	void goToOpenPosition(void);
	void goToPresetPosition(bool val);

    print_labeled_spinbox *servo_ID_spinbox;
    slider_with_box *slider_box_pos, *slider_box_vel;
    DataPlot *plot;
    std::string servo_name;
    int com_open_result, servo_ID;
    QRadioButton *enable_button;
    QPushButton *goToButton;
    unsigned int goal_servo_pos_uint, goal_servo_vel_uint;
    unsigned int actual_servo_pos_uint, actual_servo_vel_uint, actual_servo_effort_uint;
    double goal_servo_pos_double, goal_servo_vel_double, actual_servo_pos_double, actual_servo_vel_double;
    float preset_angle_1, preset_angle_2, pos_min, pos_max;
    std::vector<double *> plot_data_vec;
    dynamixel_servo_programmer *servo_programmer;
    bool isDraggedFlag, enabled_flag;
    QMutex *servo_mutex;
	QLabel *background_box;

	bool last_end_position;

private Q_SLOTS:
    void change_enabling(bool checked);
    void pos_or_vel_slider_changed(void);
    void goToPresetToggleButtonFunction(void);
    void change_servo_ID(double new_servo_ID);
};



#endif
