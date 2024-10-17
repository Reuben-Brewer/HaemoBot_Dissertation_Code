#ifndef MY_STEPPER_PROGRAMMER_H
#define MY_STEPPER_PROGRAMMER_H

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

class my_stepper_programmer: public QWidget
{
    Q_OBJECT
public:
    my_stepper_programmer(QWidget *parent, QString name_in = "");
    void timer_update(QTimerEvent *e);
    void readMessage(std::vector<signed char> & stepper_homed_state, std::vector<double> & stepper_actual_pos, double & BP_analog, bool & message_done_flag);
    void home_steppers(std::vector<bool> motors_home, std::vector<unsigned char> motors_home_speed);
    void move_stepper(std::vector<double> goal_pos_vec, std::vector<double> goal_vel_vec, std::vector<bool> move_mode, double BP_goal_pressure);
    void send_serial(std::vector<unsigned int> message_vec);
    bool isManualControl(void);
    bool isJointCoords(void);
	void updateOmniInput(std::vector<double> omni_pos_vec_in);
	void updateDesiredPosVelAccel(void);
	void updateActualPos(std::vector<double> PosActual_in);
	double saturateValue(double val_in, double max, double min);
	void set_manual_control_flag(bool val);
	void close(void);

    std::string name;
    int stepper_ID, com_open_result;

    bool Tx_enabled;
    bool manual_control_flag, last_manual_control_flag;
    bool joint_coords_flag;
    print_labeled_spinbox *servo_target_ID_spinbox, *servo_new_ID_spinbox, *servo_compliance_spinbox, *servo_target_baud_spinbox, *servo_new_baud_spinbox;
    QPushButton *home_all_steppers_button, *rezero_omni_button, *connect_serial_button;
    QRadioButton *enable_Tx_button, *manual_control_button, *joint_coords_button;
    rlSerial *comPortStepper;
    int message_to_be_sent;
    float debug_from_micro;

    //double manualX, manualY, manualZ;
	std::vector<double> PosDesired, PosActual, Pos_omniInput_raw, Pos_omniInput_rezero_snapshot, Pos_omniInput_offset, Pos_manualSliderInput;
    bool isDraggedFlag;
    slider_with_box *slider_box_manual_x, *slider_box_manual_y, *slider_box_manual_z;
	QLabel *background_box, *X_desired_label, *Y_desired_label, *Z_desired_label;
	int comPortStepperResult;

    double Xmin, Xmax, Xinc, Xinit, Ymin, Ymax, Yinc, Yinit, Zmin, Zmax, Zinc, Zinit;
	bool manual_control_button_needs_to_be_checked, haptic_control_button_needs_to_be_checked;

private Q_SLOTS:
    void home_all_steppers(void);
    void change_Tx_enabling(bool checked);
    void change_manual_control(bool checked);
    void x_slider_changed(void);
    void y_slider_changed(void);
    void z_slider_changed(void);
    void change_joint_coords(bool checked);
	void take_rezero_omni_snapshot(void);
	void connect_serial(void);
};



#endif
