#ifndef NEEDLEBOT_CONTROLLER_H
#define NEEDLEBOT_CONTROLLER_H

#include "slider_with_box.h"
#include <string>
#include <vector>
#include <QRadioButton>
#include <QPushButton>
#include "data_plot.h"
#include <QMutex>
#include <QLabel>
#include "print_labeled_spinbox.h"
#include "robot_defines.h"
#include <cml/cml.h>
#include <QString>


class needleBot_controller: public QWidget
{
    Q_OBJECT
public:
    needleBot_controller(QWidget *parent, QString name_in = "");
    void timer_update(QTimerEvent *e);
    bool isManualControl(void);
	void updateDesiredPosVelAccel(void);
	void updateOmniInput(std::vector<double> omni_pos_vec_in);
	void updateActualPosVelAccel(std::vector<double> PosActual_in, std::vector<double> VelActual_in, std::vector<double> AccelActual_in, double current_global_time_in);
	void calculate_robot_line(void);
	void updateInsertionForcePopState(double insertion_force_pop_detection_state_latched_in);
	bool shouldOutsideThreadsYieldControlToNeedleBotController(void);
	void updateAssistoBotForce(std::vector<double> assistoBot_ATI_force_vec_in);
	void set_manual_control_flag(bool val);
	void set_haptic_control_flag(bool val);
	void set_auto_control_flag(bool val);

    std::string name;
    bool manual_control_flag, haptic_control_flag, auto_control_flag, last_manual_control_flag, last_haptic_control_flag, last_auto_control_flag;
    QPushButton *home_button, *rezero_omni_button, *auto_line_snapshot_button;
    QRadioButton *manual_control_button, *haptic_control_button, *auto_control_button;
    bool isDraggedFlag;
    slider_with_box *slider_box_manual_x, *slider_box_manual_y, *slider_box_manual_z, *slider_box_manual_yaw, *slider_box_manual_pitch, *slider_box_manual_roll, *slider_box_manual_cathins;

    double Xmin_SliderManual, Xmax_SliderManual, Xinc_SliderManual, Xinit_SliderManual;
    double Ymin_SliderManual, Ymax_SliderManual, Yinc_SliderManual, Yinit_SliderManual;
    double Zmax_SliderManual, Zinc_SliderManual, Zinit_SliderManual, Zmin_SliderManual;
    double YAWmax_SliderManual, YAWinc_SliderManual, YAWinit_SliderManual, YAWmin_SliderManual;
    double PITCHmax_SliderManual, PITCHinc_SliderManual , PITCHinit_SliderManual, PITCHmin_SliderManual;
    double ROLLmax_SliderManual, ROLLinc_SliderManual, ROLLinit_SliderManual, ROLLmin_SliderManual;
    double CATHINSmax_SliderManual, CATHINSinc_SliderManual, CATHINSinit_SliderManual, CATHINSmin_SliderManual;

	std::vector<double> PosActual, VelActual, AccelActual, PosDesired, VelDesired, AccelDesired, Pos_omniInput_raw, Pos_omniInput_rezero_snapshot, Pos_omniInput_offset, Vel_omniInput, Pos_manualSliderInput, Pos_auto_line_snapshot, Pos_auto_line_endpoint, Pos_auto_line_endpoint_AFTER_POP, Pos_auto_line_backup_to_this_point_at_end;
	QLabel *X_label, *Y_label, *Z_label, *Yaw_label, *Pitch_label, *Roll_label, *CathIns_label, *background_box, *box_title, *auto_line_direction_label, *auto_insertion_substate_label, *insertion_depth_label, *assistoBot_pos_Actual_label, *assistoBot_pos_Desired_label, *debug_label, *needleBot_pos_EndPoint_label, *assistoBot_hand_contact_force_Z_label, *assistoBot_hand_contact_traction_force_label, *insertion_site_label;

	double current_global_time, time_auto_line_snapshot, auto_line_insertion_depth, auto_line_insertion_depth_max, auto_line_insertion_time_limit, auto_line_insertion_time_limit_max, insertion_force_pop_detection_state_latched, auto_line_insertion_depth_AFTER_POP, auto_line_insertion_depth_max_AFTER_POP, auto_line_insertion_time_limit_AFTER_POP, auto_line_insertion_time_limit_max_AFTER_POP;
	double auto_line_CATH_insertion_depth, auto_line_CATH_insertion_depth_max, auto_line_CATH_insertion_time_limit, auto_line_CATH_insertion_time_limit_max;
	print_labeled_spinbox *auto_line_insertion_depth_spinbox, *auto_line_insertion_time_limit_spinbox, *auto_line_CATH_insertion_depth_spinbox, *auto_line_CATH_insertion_time_limit_spinbox, *auto_line_insertion_depth_AFTER_POP_spinbox, *auto_line_insertion_time_limit_AFTER_POP_spinbox, *assistoBot_hand_contact_force_Z_threshold_spinbox, *assistoBot_hand_contact_traction_force_threshold_spinbox;
	cml::vector3f p_line_robot, line_dir_robot, line_dir_robot_auto_line_snapshot;    
	cml::matrix33f_c rot_matrix;
	double CathLoopState, CathSwingArmState;
	double time_into_auto_insertion, time_into_auto_insertion_offset;
	int auto_insertion_substate, auto_is_actively_inserting_flag;
	std::vector<double> auto_insertion_substate_time_vector;
	double insertion_depth_before_detecting_pop, current_insertion_depth;
	std::vector<double> auto_line_assistoBot_posDesired_vec, auto_line_assistoBot_posActual_vec, Pos_assistoBot_auto_line_snapshot;
	bool servo_2_ethanol_PosDesired, servo_3_air_PosDesired, auto_line_snapshot_has_been_taken;
	int what_needs_to_happen_to_data_logger_file;
	std::vector<double> assistoBot_ATI_force_vec, Pos_assistoBot_desired_insertion_site;
	std::vector<QString> auto_insertion_substate_string_descriptor_vec;
	double assistoBot_number_of_cleaning_horizontal_sweeps_N, assistoBot_cleaning_horizontal_sweeps_total_time_allowed_T, assistoBot_cleaning_horizontal_sweeps_amplitude_A, assistoBot_z_drop_velocity, assistoBot_Z_axis_force_limit, assistoBot_traction_goal, assistoBot_ATI_human_signal_to_let_go_limit;
	int what_needs_to_happen_to_foot_pedal_focus;
	bool manual_control_button_needs_to_be_checked, haptic_control_button_needs_to_be_checked, auto_control_button_needs_to_be_checked;
	double assistoBot_hand_contact_force_Z, assistoBot_hand_contact_force_Z_threshold, assistoBot_hand_contact_force_Z_threshold_max, assistoBot_hand_contact_traction_force, assistoBot_hand_contact_traction_force_threshold, assistoBot_hand_contact_traction_force_threshold_max;
	bool assistoBot_hand_contact_force_Z_THRESHOLD_EXCEEDED_BOOL, assistoBot_hand_contact_traction_force_THRESHOLD_EXCEEDED_BOOL;
	double auto_line_end_of_procedure_needle_retraction_distance, auto_line_needleBot_backs_up_at_start_to_give_room_distance;
	std::vector<double> Pos_auto_line_needleBot_backs_up_at_start_to_give_room;

private Q_SLOTS:
    void home(void);
    void x_slider_changed(void);
    void y_slider_changed(void);
    void z_slider_changed(void);
	void yaw_slider_changed(void);
    void pitch_slider_changed(void);
    void roll_slider_changed(void);
	void cathins_slider_changed(void);
	void take_rezero_omni_snapshot(void);
	void take_auto_line_snapshot(void);
};



#endif
