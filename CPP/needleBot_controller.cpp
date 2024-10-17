#include "needleBot_controller.h"

union my_union
{
    float float_num;
    char char_num[4];
};

needleBot_controller::needleBot_controller(QWidget *parent, QString name_in):
    QWidget(parent)
{
    name = name_in.toStdString();
  
	PosActual = std::vector<double>(7,0);
	VelActual = std::vector<double>(7,0);
	AccelActual = std::vector<double>(7,0);
	PosDesired = std::vector<double>(7,0);
	VelDesired = std::vector<double>(7,0);
	AccelDesired = std::vector<double>(7,0);
	Pos_omniInput_raw = std::vector<double>(7,0); 
	Pos_omniInput_rezero_snapshot = std::vector<double>(7,0);
	Pos_omniInput_offset = std::vector<double>(7,0);
	Vel_omniInput = std::vector<double>(7,0);
	Pos_manualSliderInput = std::vector<double>(7,0);
	Pos_auto_line_snapshot = std::vector<double>(7,0);
	Pos_auto_line_endpoint = std::vector<double>(7,0);
	Pos_auto_line_endpoint_AFTER_POP = std::vector<double>(7,0);
	Pos_auto_line_backup_to_this_point_at_end = std::vector<double>(7,0);
	Pos_auto_line_needleBot_backs_up_at_start_to_give_room = std::vector<double>(7,0);
	CathLoopState = 0; //starts open
	CathSwingArmState = 0; //starts open
	auto_insertion_substate_time_vector = std::vector<double>(33,0); ///////////////////////////////////////////////////////////////////////// WILL NEED TO MODIFY THIS SIZE OFTEN
	auto_insertion_substate_string_descriptor_vec = std::vector<QString>(33,"");
	auto_insertion_substate = 0; //WAS ORIGINALLY -1, BUT THIS MAY HAVE BEEN CAUSING THE VECTOR INDEXING ISSUES. CHANGING TO 0 MADE THEM DISAPPEAR.
	time_into_auto_insertion = 0;
	auto_is_actively_inserting_flag = 0;
	insertion_depth_before_detecting_pop = 0;
	current_insertion_depth = 0;
	auto_line_assistoBot_posDesired_vec = std::vector<double>(3,0);
	auto_line_assistoBot_posActual_vec = std::vector<double>(3,0);
	Pos_assistoBot_desired_insertion_site = std::vector<double>(3,-1);
	Pos_assistoBot_auto_line_snapshot = std::vector<double>(3,0);
	servo_2_ethanol_PosDesired = 0; //open
	servo_3_air_PosDesired = 0; //open
	auto_line_snapshot_has_been_taken = 0;
	what_needs_to_happen_to_data_logger_file = 0;
	assistoBot_ATI_force_vec = std::vector<double>(6,0); 
	time_into_auto_insertion_offset = 0;
	assistoBot_ATI_human_signal_to_let_go_limit = 0;
	what_needs_to_happen_to_foot_pedal_focus = 0;
	manual_control_button_needs_to_be_checked = 0;
	haptic_control_button_needs_to_be_checked = 0;
	auto_control_button_needs_to_be_checked = 0;
	
	/////////////////////////background_box must be the first GUI object so that it doesn't cover over other objects!!!
	background_box = new QLabel("",this);
	background_box->setGeometry(0,0,690,675);
	background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(rgb(200,255,200))}");

	box_title = new QLabel("NeedleBot Controller",this);
	box_title->setGeometry(5,5,150,30);
	box_title->setFont(QFont("Times", 12, QFont::Bold));

    home_button = new QPushButton("Home NeedleBot", this);
    home_button->setGeometry(QRect(5, 40, 100, 20));
    connect(home_button, SIGNAL(clicked()), this, SLOT(home()));

	rezero_omni_button = new QPushButton("Rezero Omni", this);
    rezero_omni_button->setGeometry(QRect(110, 40, 100, 20));
    connect(rezero_omni_button, SIGNAL(clicked()), this, SLOT(take_rezero_omni_snapshot()));

	manual_control_flag = 1;
	last_manual_control_flag = 1;
    manual_control_button = new QRadioButton("Manual Control", this);
    manual_control_button->setGeometry(QRect(220,5,100,20));
    manual_control_button->setAutoExclusive(1);
    manual_control_button->setChecked(manual_control_flag);

	haptic_control_flag = 0;
	last_haptic_control_flag = 0;
    haptic_control_button = new QRadioButton("Haptic Control", this);
    haptic_control_button->setGeometry(QRect(220,30,100,20));
    haptic_control_button->setAutoExclusive(1);
    haptic_control_button->setChecked(haptic_control_flag);

    auto_control_flag = 0;
	last_auto_control_flag = 0;
    auto_control_button = new QRadioButton("Auto Control", this);
    auto_control_button->setGeometry(QRect(220,55,100,20));
    auto_control_button->setAutoExclusive(1);
    auto_control_button->setChecked(auto_control_flag);

	int labels_start_width = 200;
	int labels_start_height = 40;
	int labels_start_height_inc = 5;

	Pos_manualSliderInput[0] = -1*rd_position_offset_for_needleBot_to_assistoBot_calibration_0;
    Xmin_SliderManual = -76;
    Xmax_SliderManual = 106;
    Xinc_SliderManual = 1.0;
    Xinit_SliderManual = Pos_manualSliderInput[0];
	PosDesired[0] = Pos_manualSliderInput[0]; //WE'RE STARTING OFF IN MANUAL CONTROL, SO THAT'S WHERE OUR STARTING DESIRED POS MUST BE

	Pos_manualSliderInput[1] = -1*rd_position_offset_for_needleBot_to_assistoBot_calibration_1;
    Ymin_SliderManual = -60;
    Ymax_SliderManual = 108;
    Yinc_SliderManual = 1.0;
    Yinit_SliderManual = Pos_manualSliderInput[1];
	PosDesired[1] = Pos_manualSliderInput[1]; //WE'RE STARTING OFF IN MANUAL CONTROL, SO THAT'S WHERE OUR STARTING DESIRED POS MUST BE

	Pos_manualSliderInput[2] = -1*rd_position_offset_for_needleBot_to_assistoBot_calibration_2;
    Zmax_SliderManual = 120.0;
    Zinc_SliderManual = 1.0;
    Zinit_SliderManual = Pos_manualSliderInput[2];
    Zmin_SliderManual = -1*rd_position_offset_for_needleBot_to_assistoBot_calibration_2;
	PosDesired[2] = Pos_manualSliderInput[2]; //WE'RE STARTING OFF IN MANUAL CONTROL, SO THAT'S WHERE OUR STARTING DESIRED POS MUST BE

	Pos_manualSliderInput[3] = -1*rd_position_offset_for_needleBot_to_assistoBot_calibration_3;
    YAWmax_SliderManual = 114.7 - 5.0;
    YAWinc_SliderManual = 1.0;
    YAWinit_SliderManual = Pos_manualSliderInput[3];
	YAWmin_SliderManual = -114.7 + 5.0;
	PosDesired[3] = Pos_manualSliderInput[3]; //WE'RE STARTING OFF IN MANUAL CONTROL, SO THAT'S WHERE OUR STARTING DESIRED POS MUST BE

	Pos_manualSliderInput[4] = -1*rd_position_offset_for_needleBot_to_assistoBot_calibration_4;
    PITCHmax_SliderManual = 92.7;
    PITCHinc_SliderManual = 1.0;
    PITCHinit_SliderManual = Pos_manualSliderInput[4];
	PITCHmin_SliderManual = -92.7;
	PosDesired[4] = Pos_manualSliderInput[4]; //WE'RE STARTING OFF IN MANUAL CONTROL, SO THAT'S WHERE OUR STARTING DESIRED POS MUST BE

	Pos_manualSliderInput[5] = -1*rd_position_offset_for_needleBot_to_assistoBot_calibration_5;
    ROLLmax_SliderManual = 1000.0;
    ROLLinc_SliderManual = 1.0;
    ROLLinit_SliderManual = Pos_manualSliderInput[5];
	ROLLmin_SliderManual = -1000.0;
	PosDesired[5] = Pos_manualSliderInput[5]; //WE'RE STARTING OFF IN MANUAL CONTROL, SO THAT'S WHERE OUR STARTING DESIRED POS MUST BE

	Pos_manualSliderInput[6] = -1*rd_position_offset_for_needleBot_to_assistoBot_calibration_6;
    CATHINSmax_SliderManual = rd_needle_driver_stringpot_PositionMax + 0.0; //The extra 0.5 allows for us to command overshoots.
    CATHINSinc_SliderManual = 1.0;
    CATHINSinit_SliderManual = Pos_manualSliderInput[6];
	CATHINSmin_SliderManual = 0.0;
	PosDesired[6] = Pos_manualSliderInput[6]; //WE'RE STARTING OFF IN MANUAL CONTROL, SO THAT'S WHERE OUR STARTING DESIRED POS MUST BE

	int slider_start_x = 5;
	int slider_start_y = 75;
	int slider_start_width = 380;
	int slider_start_width_inc = 5;
	int slider_start_height = 60;
	int slider_start_height_inc = 5;

    slider_box_manual_x = new slider_with_box(this, "manual X", 0, Xmin_SliderManual, Xmax_SliderManual, Xinc_SliderManual, Xinit_SliderManual);
    slider_box_manual_x->setGeometry(QRect(slider_start_x, slider_start_y + 0*(slider_start_height + slider_start_height_inc), slider_start_width, slider_start_height));
    connect(slider_box_manual_x, SIGNAL(valueChanged(double)), this, SLOT(x_slider_changed(void)));

	X_label = new QLabel("", this);
    X_label->setGeometry(QRect(slider_start_width + 5, slider_start_y + 0*(slider_start_height + slider_start_height_inc), labels_start_width, labels_start_height));

    slider_box_manual_y = new slider_with_box(this, "manual Y", 0, Ymin_SliderManual, Ymax_SliderManual, Yinc_SliderManual, Yinit_SliderManual);
    slider_box_manual_y->setGeometry(QRect(slider_start_x, slider_start_y + 1*(slider_start_height + slider_start_height_inc), slider_start_width, slider_start_height));
    connect(slider_box_manual_y, SIGNAL(valueChanged(double)), this, SLOT(y_slider_changed(void)));

	Y_label = new QLabel("", this);
    Y_label->setGeometry(QRect(slider_start_width + 5, slider_start_y + 1*(slider_start_height + slider_start_height_inc), labels_start_width, labels_start_height));

    slider_box_manual_z = new slider_with_box(this, "manual Z", 0, Zmin_SliderManual, Zmax_SliderManual, Zinc_SliderManual, Zinit_SliderManual);
    slider_box_manual_z->setGeometry(QRect(slider_start_x, slider_start_y + 2*(slider_start_height + slider_start_height_inc), slider_start_width, slider_start_height));
    connect(slider_box_manual_z, SIGNAL(valueChanged(double)), this, SLOT(z_slider_changed(void)));

	Z_label = new QLabel("", this);
    Z_label->setGeometry(QRect(slider_start_width + 5, slider_start_y + 2*(slider_start_height + slider_start_height_inc), labels_start_width, labels_start_height));

    slider_box_manual_yaw = new slider_with_box(this, "manual YAW", 0, YAWmin_SliderManual, YAWmax_SliderManual, YAWinc_SliderManual, YAWinit_SliderManual);
    slider_box_manual_yaw->setGeometry(QRect(slider_start_x, slider_start_y + 3*(slider_start_height + slider_start_height_inc), slider_start_width, slider_start_height));
    connect(slider_box_manual_yaw, SIGNAL(valueChanged(double)), this, SLOT(yaw_slider_changed(void)));

	Yaw_label = new QLabel("", this);
    Yaw_label->setGeometry(QRect(slider_start_width + 5, slider_start_y + 3*(slider_start_height + slider_start_height_inc), labels_start_width, labels_start_height));

	slider_box_manual_pitch = new slider_with_box(this, "manual PITCH", 0, PITCHmin_SliderManual, PITCHmax_SliderManual, PITCHinc_SliderManual, PITCHinit_SliderManual);
    slider_box_manual_pitch->setGeometry(QRect(slider_start_x, slider_start_y + 4*(slider_start_height + slider_start_height_inc), slider_start_width, slider_start_height));
    connect(slider_box_manual_pitch, SIGNAL(valueChanged(double)), this, SLOT(pitch_slider_changed(void)));

	Pitch_label = new QLabel("", this);
    Pitch_label->setGeometry(QRect(slider_start_width + 5, slider_start_y + 4*(slider_start_height + slider_start_height_inc), labels_start_width, labels_start_height));

	slider_box_manual_roll = new slider_with_box(this, "manual ROLL", 0, ROLLmin_SliderManual, ROLLmax_SliderManual, ROLLinc_SliderManual, ROLLinit_SliderManual);
    slider_box_manual_roll->setGeometry(QRect(slider_start_x, slider_start_y + 5*(slider_start_height + slider_start_height_inc), slider_start_width, slider_start_height));
    connect(slider_box_manual_roll, SIGNAL(valueChanged(double)), this, SLOT(roll_slider_changed(void)));

	Roll_label = new QLabel("", this);
    Roll_label->setGeometry(QRect(slider_start_width + 5, slider_start_y + 5*(slider_start_height + slider_start_height_inc), labels_start_width, labels_start_height));

	slider_box_manual_cathins = new slider_with_box(this, "manual CATH INS", 0, CATHINSmin_SliderManual, CATHINSmax_SliderManual, CATHINSinc_SliderManual, CATHINSinit_SliderManual);
    slider_box_manual_cathins->setGeometry(QRect(slider_start_x, slider_start_y + 6*(slider_start_height + slider_start_height_inc), slider_start_width, slider_start_height));
    connect(slider_box_manual_cathins, SIGNAL(valueChanged(double)), this, SLOT(cathins_slider_changed(void)));

	CathIns_label = new QLabel("", this);
    CathIns_label->setGeometry(QRect(slider_start_width + 5, slider_start_y + 6*(slider_start_height + slider_start_height_inc), labels_start_width, labels_start_height));

	int spinbox_start_x = 530;
	int spinbox_start_y = 5; 
	int spinbox_start_width = 150;
	int spinbox_start_width_inc = 5;
	int spinbox_start_height = 25;
	int spinbox_start_height_inc = 5;

	auto_line_insertion_depth = 15.0;
	auto_line_insertion_depth_max = 25.0;
	auto_line_insertion_depth_spinbox = new print_labeled_spinbox(this, "Line Depth: ", 0.001, 3, 0.0, auto_line_insertion_depth_max, auto_line_insertion_depth);
	auto_line_insertion_depth_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 0*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));
	
	auto_line_insertion_time_limit = 2.0;
	auto_line_insertion_time_limit_max = 20.0;
	auto_line_insertion_time_limit_spinbox = new print_labeled_spinbox(this, "Line Time: ", 0.001, 3, 0.0, auto_line_insertion_time_limit_max, auto_line_insertion_time_limit);
	auto_line_insertion_time_limit_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 1*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));

	auto_line_insertion_depth_AFTER_POP = 1.0;
	auto_line_insertion_depth_max_AFTER_POP = 20.0;
	auto_line_insertion_depth_AFTER_POP_spinbox = new print_labeled_spinbox(this, "Aft Pop Line Depth: ", 0.001, 3, 0.0, auto_line_insertion_depth_max_AFTER_POP, auto_line_insertion_depth_AFTER_POP);
	auto_line_insertion_depth_AFTER_POP_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 2*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));
	
	auto_line_insertion_time_limit_AFTER_POP = 1.0;
	auto_line_insertion_time_limit_max_AFTER_POP = 20.0;
	auto_line_insertion_time_limit_AFTER_POP_spinbox = new print_labeled_spinbox(this, "Aft Pop Line Time: ", 0.001, 3, 0.0, auto_line_insertion_time_limit_max_AFTER_POP, auto_line_insertion_time_limit_AFTER_POP);
	auto_line_insertion_time_limit_AFTER_POP_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 3*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));

	auto_line_CATH_insertion_depth = 20.0;
	auto_line_CATH_insertion_depth_max = 40.0;
	auto_line_CATH_insertion_depth_spinbox = new print_labeled_spinbox(this, "Cath Depth: ", 0.001, 3, 0.0, auto_line_CATH_insertion_depth_max, auto_line_CATH_insertion_depth);
	auto_line_CATH_insertion_depth_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 4*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));
	
	auto_line_CATH_insertion_time_limit = 2.0;
	auto_line_CATH_insertion_time_limit_max = 15.0;
	auto_line_CATH_insertion_time_limit_spinbox = new print_labeled_spinbox(this, "Cath Time: ", 0.001, 3, 0.0, auto_line_CATH_insertion_time_limit_max, auto_line_CATH_insertion_time_limit);
	auto_line_CATH_insertion_time_limit_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 5*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));

	assistoBot_hand_contact_force_Z_threshold = 1.0;
	assistoBot_hand_contact_force_Z_threshold_max = 10.0;
	assistoBot_hand_contact_force_Z_threshold_spinbox = new print_labeled_spinbox(this, "Fz Thresh: ", 0.001, 3, 0.0, assistoBot_hand_contact_force_Z_threshold_max, assistoBot_hand_contact_force_Z_threshold);
	assistoBot_hand_contact_force_Z_threshold_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 6*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));

	assistoBot_hand_contact_force_Z_label = new QLabel("", this);
    assistoBot_hand_contact_force_Z_label->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 7*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, 2*spinbox_start_height));

	assistoBot_hand_contact_traction_force_threshold = 1.0;
	assistoBot_hand_contact_traction_force_threshold_max = 10.0;
	assistoBot_hand_contact_traction_force_threshold_spinbox = new print_labeled_spinbox(this, "Tract Thresh: ", 0.001, 3, 0.0, assistoBot_hand_contact_traction_force_threshold_max, assistoBot_hand_contact_traction_force_threshold);
	assistoBot_hand_contact_traction_force_threshold_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 9*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));

	assistoBot_hand_contact_traction_force_label = new QLabel("", this);
    assistoBot_hand_contact_traction_force_label->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 10*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, 2*spinbox_start_height));

	int auto_insertion_ino_labels_start_x = 5;
	int auto_insertion_ino_labels_start_y = slider_start_y + 7*(slider_start_height + slider_start_height_inc);
	int auto_insertion_ino_labels_width = 130;
	int auto_insertion_ino_labels_width_inc = 5;
	int auto_insertion_ino_labels_height = 25;
	int auto_insertion_ino_labels_height_inc = 5;

	insertion_site_label = new QLabel("", this);
    insertion_site_label->setGeometry(QRect(auto_insertion_ino_labels_start_x + 0*(auto_insertion_ino_labels_width + auto_insertion_ino_labels_width_inc), auto_insertion_ino_labels_start_y + 0*(auto_insertion_ino_labels_height + auto_insertion_ino_labels_height_inc), spinbox_start_width, 70));

	auto_line_direction_label = new QLabel("", this);
    auto_line_direction_label->setGeometry(QRect(auto_insertion_ino_labels_start_x + 1*(auto_insertion_ino_labels_width + auto_insertion_ino_labels_width_inc), auto_insertion_ino_labels_start_y + 0*(auto_insertion_ino_labels_height + auto_insertion_ino_labels_height_inc), spinbox_start_width, 70));

	assistoBot_pos_Actual_label = new QLabel("", this);
    assistoBot_pos_Actual_label->setGeometry(QRect(auto_insertion_ino_labels_start_x + 2*(auto_insertion_ino_labels_width + auto_insertion_ino_labels_width_inc), auto_insertion_ino_labels_start_y + 0*(auto_insertion_ino_labels_height + auto_insertion_ino_labels_height_inc), 110, 70));

	assistoBot_pos_Desired_label = new QLabel("", this);
    assistoBot_pos_Desired_label->setGeometry(QRect(auto_insertion_ino_labels_start_x + 3*(auto_insertion_ino_labels_width + auto_insertion_ino_labels_width_inc), auto_insertion_ino_labels_start_y + 0*(auto_insertion_ino_labels_height + auto_insertion_ino_labels_height_inc), 110, 70));

	needleBot_pos_EndPoint_label = new QLabel("", this);
    needleBot_pos_EndPoint_label->setGeometry(QRect(auto_insertion_ino_labels_start_x + 4*(auto_insertion_ino_labels_width + auto_insertion_ino_labels_width_inc), auto_insertion_ino_labels_start_y + 0*(auto_insertion_ino_labels_height + auto_insertion_ino_labels_height_inc), 110, 70));

	auto_line_snapshot_button = new QPushButton("Auto Line Snapshot", this);
    auto_line_snapshot_button->setGeometry(QRect(auto_insertion_ino_labels_start_x + 0*(auto_insertion_ino_labels_width + auto_insertion_ino_labels_width_inc), auto_insertion_ino_labels_start_y + 2*(auto_insertion_ino_labels_height + auto_insertion_ino_labels_height_inc)+10, 125, spinbox_start_height));
    connect(auto_line_snapshot_button, SIGNAL(clicked()), this, SLOT(take_auto_line_snapshot()));

	debug_label = new QLabel("", this);
    debug_label->setGeometry(QRect(auto_insertion_ino_labels_start_x + 0*(auto_insertion_ino_labels_width + auto_insertion_ino_labels_width_inc), auto_insertion_ino_labels_start_y + 3*(auto_insertion_ino_labels_height + auto_insertion_ino_labels_height_inc), spinbox_start_width, 70));

	insertion_depth_label = new QLabel("", this);
    insertion_depth_label->setGeometry(QRect(auto_insertion_ino_labels_start_x + 1*(auto_insertion_ino_labels_width + auto_insertion_ino_labels_width_inc), auto_insertion_ino_labels_start_y + 2*(auto_insertion_ino_labels_height + auto_insertion_ino_labels_height_inc), spinbox_start_width, 40));

	auto_insertion_substate_label = new QLabel("", this);
    auto_insertion_substate_label->setGeometry(QRect(auto_insertion_ino_labels_start_x + 1*(auto_insertion_ino_labels_width + auto_insertion_ino_labels_width_inc), auto_insertion_ino_labels_start_y + 3*(auto_insertion_ino_labels_height + auto_insertion_ino_labels_height_inc), 525, 40));

    isDraggedFlag = false;

	
}

void needleBot_controller::updateActualPosVelAccel(std::vector<double> PosActual_in, std::vector<double> VelActual_in, std::vector<double> AccelActual_in, double current_global_time_in)
{
	PosActual = PosActual_in;
	VelActual = VelActual_in;
	AccelActual = AccelActual_in;
	current_global_time = current_global_time_in;
}

void needleBot_controller::updateDesiredPosVelAccel(void)
{
	if(manual_control_flag == 1 && haptic_control_flag == 0 && auto_control_flag == 0)
	{
		for(int i = 0; i < PosDesired.size(); i++)
		{
			PosDesired[i] = Pos_manualSliderInput[i];
			VelDesired[i] = 0; 
			AccelDesired[i] = 0;
		}
	}
	else if(manual_control_flag == 0 && haptic_control_flag == 1 && auto_control_flag == 0)
	{
		for(int i = 0; i < PosDesired.size(); i++)
		{
			PosDesired[i] = Pos_omniInput_raw[i] - Pos_omniInput_rezero_snapshot[i];
			VelDesired[i] = Vel_omniInput[i];
			AccelDesired[i] = 0;
		}
	}
	else if(manual_control_flag == 0 && haptic_control_flag == 0 && auto_control_flag == 1 && auto_is_actively_inserting_flag == 1)
	{
		if(auto_line_snapshot_has_been_taken == 1)
		{
			///////////////////////////////////////////////////////////////////////////////////
			time_into_auto_insertion = current_global_time - time_auto_line_snapshot;

			double time_sum = 0;
			auto_insertion_substate = 0; //WAS ORIGINALLY -1, BUT THIS MAY HAVE BEEN CAUSING THE VECTOR INDEXING ISSUES. CHANGING TO 0 MADE THEM DISAPPEAR.
			for(int substate_index = 0; substate_index < auto_insertion_substate_time_vector.size(); substate_index++)
			{
				time_sum = time_sum + auto_insertion_substate_time_vector[substate_index];
				if(time_into_auto_insertion < time_sum)
				{
					auto_insertion_substate = substate_index;
					time_into_auto_insertion_offset = time_sum - auto_insertion_substate_time_vector[substate_index];
					break;
				}
			}
			///////////////////////////////////////////////////////////////////////////////////

			/*
					[0] = "ASSISTOBOT: Move far away from the hand.";
					[1] = "ND: Close the cath swing arm.";
					[2] = "ND: Close the cath loop.";
					[3] = "NEEDLExyz: Back the needle far away to give assistoBot room.";
					[4] = "ASSISTOBOT: Move XY a little to left of insertion site for IPA spraying.";
					[5] = "ASSISTOBOT: Move Z just above the insertion site.";
					[6] = "DYNAMIXEL: Close the IPA (spray it).";	
					[7] = "DYNAMIXEL: Open the IPA (stop spraying it).";
					[8] = "ASSISTOBOT: Move XY right above insertion site.";
					[9] = "ASSISTOBOT: Drop Z until the force sensor reads above a certain level.";
					[10] = "ASSISTOBOT: Scrub insertion site.";
					[11] = "ASSISTOBOT: Move Z just above the insertion site.";
					[12] = "DYNAMIXEL: Close the air (spray it).";
					[13] = "DYNAMIXEL: Open the air (stop spraying it).";
					[14] = "ASSISTOBOT: Move to just behind the insertion site in preparation for applying traction.";
					[15] = "ASSISTOBOT: Drop Z until the force sensor reads above a certain level";
					[16] = "ASSISTOBOT: Move backwards to apply traction.";
					[17] = "NEEDLExyz: Insert needle forward to original insertion point.";
					[18] = "NEEDLExyz: Wait for a little to let us know we're about to insert.";			
					[19] = "NEEDLExyz: Insert needle forward along auto line until sensing the pop.";
					[20] = "NEEDLExyz: Waiting time in between the pop and the further insertion into lumen.";
					[21] = "NEEDLExyz: Insert needle forward along auto line a little beyond pop to get cath into lumen.";
					[22] = "NEEDLExyz: Insert the catheter forward.";
					[23] = "ASSISTOBOT: Move Z up off of hand (release traction).";
					[24] = "ASSISTOBOT: Move XY to just behind the insertion site to prepare for clamping the cath.";
					[25] = "ASSISTOBOT: Drop Z until the force sensor reads above a certain level, clamp cath.";
					[26] = "ND: Open the cath loop.";
					[27] = "ND: Open the cath swing arm.";
					[28] = "ND: Retract the catheter backward.";
					[29] = "NEEDLExyz: Retract the needle backward.";
					[30] = "NEEDLExyz: Go far away from hand.";
					[31] = "ASSISTOBOT: Wait for human inspection while holding catheter securely.";
					[32] = "ASSISTOBOT: Go far away from hand.";
			*/


			///////////////////////////////////////////////////////////////////////////////////
			if(auto_insertion_substate == 0) //[0] = "ASSISTOBOT: Move far away from the hand.";
			{
				
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 1) //[1] = "ND: Close the cath swing arm.";
			{
				CathSwingArmState = 1;
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 2) //[2] = "ND: Close the cath loop.";
			{
				CathLoopState = 1; 
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 3) //[3] = "NEEDLExyz: Back the needle far away to give assistoBot room.";
			{
				for(int i = 0; i <= 2; i++)
				{
					current_insertion_depth = (auto_line_needleBot_backs_up_at_start_to_give_room_distance/auto_insertion_substate_time_vector[auto_insertion_substate])*(time_into_auto_insertion-time_into_auto_insertion_offset);
					PosDesired[i] = Pos_auto_line_snapshot[i] - current_insertion_depth*line_dir_robot_auto_line_snapshot[i]; 
					VelDesired[i] = 0;
					AccelDesired[i] = 0;
				}

				//ORIGINAL CODE REQUIREDE FOR CORRECT ROBOT OPERATION
				/*for(int j = 3; j <= 6; j++)
				{
					PosDesired[j] = Pos_auto_line_snapshot[j];
					VelDesired[j] = 0;
					AccelDesired[j] = 0;
				}*/ 
				//ORIGINAL CODE REQUIREDE FOR CORRECT ROBOT OPERATION

				PosDesired[3] = Pos_auto_line_snapshot[3] - ((Pos_auto_line_snapshot[3]-108)/auto_insertion_substate_time_vector[auto_insertion_substate])*(time_into_auto_insertion-time_into_auto_insertion_offset); //Set yaw to 0 at end. //ONLY FOR RCM MOVIE
				PosDesired[4] = Pos_auto_line_snapshot[4] - ((Pos_auto_line_snapshot[4]+90)/auto_insertion_substate_time_vector[auto_insertion_substate])*(time_into_auto_insertion-time_into_auto_insertion_offset); //Set pitch to 0 at end. //ONLY FOR RCM MOVIE
				PosDesired[5] = Pos_auto_line_snapshot[5] - ((Pos_auto_line_snapshot[5]+1000)/auto_insertion_substate_time_vector[auto_insertion_substate])*(time_into_auto_insertion-time_into_auto_insertion_offset); //Set ROLL to 0 at end. //ONLY FOR RCM MOVIE
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 4) //[4] = "ASSISTOBOT: Move XY a little to left of insertion site for IPA spraying.";
			{
				auto_line_assistoBot_posDesired_vec[0] = Pos_assistoBot_desired_insertion_site[0] - 40; //to left of insertion site in X
				auto_line_assistoBot_posDesired_vec[1] = Pos_assistoBot_desired_insertion_site[1] - 0;
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 5) //[5] = "ASSISTOBOT: Move Z just above the insertion site.";
			{
				auto_line_assistoBot_posDesired_vec[2] = Pos_assistoBot_desired_insertion_site[2] + 30;
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 6) //[6] = "DYNAMIXEL: Close the IPA (spray it).";	
			{
				servo_2_ethanol_PosDesired = 1; //closed
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 7) //[7] = "DYNAMIXEL: Open the IPA (stop spraying it).";
			{
				servo_2_ethanol_PosDesired = 0; //open
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 8) //[8] = "ASSISTOBOT: Move XY right above insertion site.";
			{
				auto_line_assistoBot_posDesired_vec[0] = Pos_assistoBot_desired_insertion_site[0] - 0;
				auto_line_assistoBot_posDesired_vec[1] = Pos_assistoBot_desired_insertion_site[1] - 0;
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 9) //[9] = "ASSISTOBOT: Drop Z until the force sensor reads above a certain level.";
			{
				if(assistoBot_hand_contact_force_Z_THRESHOLD_EXCEEDED_BOOL == 0)
				{
					auto_line_assistoBot_posDesired_vec[2] = Pos_assistoBot_desired_insertion_site[2] - 20; //JUST BELOW INSERTION SITE SO WE DON'T GO TOO FAR IF WE MISS THE FORCE SIGNAL
				}
				else
				{
					auto_line_assistoBot_posDesired_vec[2] = auto_line_assistoBot_posActual_vec[2]; //STOP WHERE YOU CURRENTLY ARE.
					auto_insertion_substate_time_vector[auto_insertion_substate] = time_into_auto_insertion-time_into_auto_insertion_offset;
					//cout << auto_line_assistoBot_posActual_vec[2] << " where I currently am" << endl;
				}
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 10) //[10] = "ASSISTOBOT: Scrub insertion site.";
			{
				auto_line_assistoBot_posDesired_vec[0] = Pos_assistoBot_desired_insertion_site[0] + assistoBot_cleaning_horizontal_sweeps_amplitude_A*cos(2*pi_define*(assistoBot_number_of_cleaning_horizontal_sweeps_N/assistoBot_cleaning_horizontal_sweeps_total_time_allowed_T)*(time_into_auto_insertion-time_into_auto_insertion_offset));
				auto_line_assistoBot_posDesired_vec[1] = Pos_assistoBot_desired_insertion_site[1] + assistoBot_cleaning_horizontal_sweeps_amplitude_A*sin(2*pi_define*(assistoBot_number_of_cleaning_horizontal_sweeps_N/assistoBot_cleaning_horizontal_sweeps_total_time_allowed_T)*(time_into_auto_insertion-time_into_auto_insertion_offset));
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 11) //[11] = "ASSISTOBOT: Move Z just above the insertion site.";
			{
				auto_line_assistoBot_posDesired_vec[2] = Pos_assistoBot_desired_insertion_site[2] + 30;
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 12) //[12] = "DYNAMIXEL: Close the air (spray it).";
			{
				servo_3_air_PosDesired = 1; //closed

			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 13) //[13] = "DYNAMIXEL: Open the air (stop spraying it).";
			{
				servo_3_air_PosDesired = 0; //open
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 14) //[14] = "ASSISTOBOT: Move to just behind the insertion site in preparation for applying traction.";
			{
				auto_line_assistoBot_posDesired_vec[0] = Pos_assistoBot_desired_insertion_site[0] - 0; //to left of insertion site in X
				auto_line_assistoBot_posDesired_vec[1] = Pos_assistoBot_desired_insertion_site[1] - 30;
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 15) //[15] = "ASSISTOBOT: Drop Z until the force sensor reads above a certain level";
			{
				if(assistoBot_hand_contact_force_Z_THRESHOLD_EXCEEDED_BOOL == 0)
				{
					auto_line_assistoBot_posDesired_vec[2] = Pos_assistoBot_desired_insertion_site[2] - 20; //JUST BELOW INSERTION SITE SO WE DON'T GO TOO FAR IF WE MISS THE FORCE SIGNAL
				}
				else
				{
					auto_line_assistoBot_posDesired_vec[2] = auto_line_assistoBot_posActual_vec[2]; //STOP WHERE YOU CURRENTLY ARE.
					auto_insertion_substate_time_vector[auto_insertion_substate] = time_into_auto_insertion-time_into_auto_insertion_offset;
					cout << auto_line_assistoBot_posActual_vec[2] << " where I currently am" << endl;
				}
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 16) //[16] = "ASSISTOBOT: Move backwards to apply traction.";
			{
				if(assistoBot_hand_contact_traction_force_THRESHOLD_EXCEEDED_BOOL == 0)
				{
					auto_line_assistoBot_posDesired_vec[1] = -45;
				}
				else
				{

					auto_line_assistoBot_posDesired_vec[0] = auto_line_assistoBot_posActual_vec[0]; //STOP WHERE YOU CURRENTLY ARE.
					auto_line_assistoBot_posDesired_vec[1] = auto_line_assistoBot_posActual_vec[1]; //STOP WHERE YOU CURRENTLY ARE.
					auto_line_assistoBot_posDesired_vec[2] = auto_line_assistoBot_posActual_vec[2]; //STOP WHERE YOU CURRENTLY ARE.
					auto_insertion_substate_time_vector[auto_insertion_substate] = time_into_auto_insertion-time_into_auto_insertion_offset;
					cout << auto_line_assistoBot_posActual_vec[1] << " where I currently am" << endl;
				}
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 17) //[17] = "NEEDLExyz: Insert needle forward to original insertion point.";
			{
				for(int i = 0; i <= 2; i++)
				{
					current_insertion_depth = (auto_line_needleBot_backs_up_at_start_to_give_room_distance/auto_insertion_substate_time_vector[auto_insertion_substate])*(time_into_auto_insertion-time_into_auto_insertion_offset);
					PosDesired[i] = Pos_auto_line_needleBot_backs_up_at_start_to_give_room[i] + current_insertion_depth*line_dir_robot_auto_line_snapshot[i]; 
					VelDesired[i] = 0;
					AccelDesired[i] = 0;
				}

				for(int j = 3; j <= 6; j++)
				{
					PosDesired[j] = Pos_auto_line_snapshot[j];
					VelDesired[j] = 0;
					AccelDesired[j] = 0;
				}
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 18) //[18] = "NEEDLExyz: Wait for a little to let us know we're about to insert.";
			{
				
			}
			///////////////////////////////////////////////////////////////////////////////////


			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 19) //[19] = "NEEDLExyz: Insert needle forward along auto line until sensing the pop.";
			{
				if(insertion_force_pop_detection_state_latched == 0)
				{
					for(int i = 0; i <= 2; i++)
					{
						
						current_insertion_depth = (auto_line_insertion_depth/auto_line_insertion_time_limit)*(time_into_auto_insertion-time_into_auto_insertion_offset);
						PosDesired[i] = Pos_auto_line_snapshot[i] + current_insertion_depth*line_dir_robot_auto_line_snapshot[i];
						VelDesired[i] = 0;
						AccelDesired[i] = 0;
					}

					for(int j = 3; j <= 6; j++)
					{
						PosDesired[j] = Pos_auto_line_snapshot[j];
						VelDesired[j] = 0;
						AccelDesired[j] = 0;
					}
					
				}
				else
				{
					insertion_depth_before_detecting_pop = current_insertion_depth;

					Pos_auto_line_endpoint[0] = PosActual[0]; //UPDATE BASED ON WHERE WE REALLY FELT THE POP
					Pos_auto_line_endpoint[1] = PosActual[1]; //UPDATE BASED ON WHERE WE REALLY FELT THE POP
					Pos_auto_line_endpoint[2] = PosActual[2]; //UPDATE BASED ON WHERE WE REALLY FELT THE POP

					Pos_auto_line_endpoint_AFTER_POP[0] = Pos_auto_line_endpoint[0] + auto_line_insertion_depth_AFTER_POP*line_dir_robot_auto_line_snapshot[0]; //UPDATE BASED ON WHERE WE REALLY FELT THE POP
					Pos_auto_line_endpoint_AFTER_POP[1] = Pos_auto_line_endpoint[1] + auto_line_insertion_depth_AFTER_POP*line_dir_robot_auto_line_snapshot[1]; //UPDATE BASED ON WHERE WE REALLY FELT THE POP
					Pos_auto_line_endpoint_AFTER_POP[2] = Pos_auto_line_endpoint[2] + auto_line_insertion_depth_AFTER_POP*line_dir_robot_auto_line_snapshot[2]; //UPDATE BASED ON WHERE WE REALLY FELT THE POP

					Pos_auto_line_backup_to_this_point_at_end[0] = Pos_auto_line_endpoint_AFTER_POP[0] - (insertion_depth_before_detecting_pop + auto_line_insertion_depth_AFTER_POP + auto_line_end_of_procedure_needle_retraction_distance)*line_dir_robot_auto_line_snapshot[0]; //UPDATE BASED ON WHERE WE REALLY FELT THE POP
					Pos_auto_line_backup_to_this_point_at_end[1] = Pos_auto_line_endpoint_AFTER_POP[1] - (insertion_depth_before_detecting_pop + auto_line_insertion_depth_AFTER_POP + auto_line_end_of_procedure_needle_retraction_distance)*line_dir_robot_auto_line_snapshot[1]; //UPDATE BASED ON WHERE WE REALLY FELT THE POP
					Pos_auto_line_backup_to_this_point_at_end[2] = Pos_auto_line_endpoint_AFTER_POP[2] - (insertion_depth_before_detecting_pop + auto_line_insertion_depth_AFTER_POP + auto_line_end_of_procedure_needle_retraction_distance)*line_dir_robot_auto_line_snapshot[2]; //UPDATE BASED ON WHERE WE REALLY FELT THE POP
					
					current_insertion_depth = 0;

					auto_insertion_substate_time_vector[auto_insertion_substate] = time_into_auto_insertion-time_into_auto_insertion_offset; //terminate this state and update the time vector.
				}
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 20) //[20] = "NEEDLExyz: Waiting time in between the pop and the further insertion into lumen.";
			{
				//Do nothing
			}

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 21) //[21] = "NEEDLExyz: Insert needle forward along auto line a little beyond pop to get cath into lumen";
			{
				for(int i = 0; i <= 2; i++)
				{
					current_insertion_depth = (auto_line_insertion_depth_AFTER_POP/auto_line_insertion_time_limit_AFTER_POP)*(time_into_auto_insertion-time_into_auto_insertion_offset);
					PosDesired[i] = Pos_auto_line_endpoint[i] + current_insertion_depth*line_dir_robot_auto_line_snapshot[i];
					VelDesired[i] = 0;
					AccelDesired[i] = 0;
				}

				for(int j = 3; j <= 6; j++)
				{
					PosDesired[j] = Pos_auto_line_snapshot[j];
					VelDesired[j] = 0;
					AccelDesired[j] = 0;
				}
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 22) //[22] = "NEEDLExyz: Insert the catheter forward.";
			{
				PosDesired[6] = Pos_auto_line_snapshot[6] + (auto_line_CATH_insertion_depth/auto_line_CATH_insertion_time_limit)*(time_into_auto_insertion-time_into_auto_insertion_offset);
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 23) //[23] = "ASSISTOBOT: Move Z up off of hand (release traction).";
			{
				auto_line_assistoBot_posDesired_vec[2] = Pos_assistoBot_desired_insertion_site[2] + 20;
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 24) //[24] = "ASSISTOBOT: Move XY to just behind the insertion site to prepare for clamping the cath.";
			{
				auto_line_assistoBot_posDesired_vec[0] = Pos_assistoBot_desired_insertion_site[0] - 0; //to left of insertion site in X
				auto_line_assistoBot_posDesired_vec[1] = Pos_assistoBot_desired_insertion_site[1] - 20;
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 25) //[25] = "ASSISTOBOT: Drop Z until the force sensor reads above a certain level, clamp cath.";
			{
				if(assistoBot_hand_contact_force_Z_THRESHOLD_EXCEEDED_BOOL == 0)
				{
					auto_line_assistoBot_posDesired_vec[2] = Pos_assistoBot_desired_insertion_site[2] - 20; //JUST BELOW INSERTION SITE SO WE DON'T GO TOO FAR IF WE MISS THE FORCE SIGNAL
				}
				else
				{
					auto_line_assistoBot_posDesired_vec[2] = auto_line_assistoBot_posActual_vec[2]; //STOP WHERE YOU CURRENTLY ARE.
					auto_insertion_substate_time_vector[auto_insertion_substate] = time_into_auto_insertion-time_into_auto_insertion_offset;
					cout << auto_line_assistoBot_posActual_vec[2] << " where I currently am" << endl;
				}
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 26) //[26] = "ND: Open the cath loop.";
			{
				CathLoopState = 0; 
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 27) //[27] = "ND: Open the cath swing arm.";
			{
				CathSwingArmState = 0;
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 28) //[28] = "ND: Retract the catheter backward.";
			{
				PosDesired[6] = Pos_auto_line_endpoint[6] - (auto_line_CATH_insertion_depth/auto_line_CATH_insertion_time_limit)*(time_into_auto_insertion-time_into_auto_insertion_offset);
			}		
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 29) //[29] = "NEEDLExyz: Retract the needle backward.";
			{
				for(int i = 0; i <= 2; i++)
				{
					PosDesired[i] = Pos_auto_line_endpoint_AFTER_POP[i] - ((insertion_depth_before_detecting_pop + auto_line_insertion_depth_AFTER_POP + auto_line_end_of_procedure_needle_retraction_distance)/(auto_line_insertion_time_limit + auto_line_insertion_time_limit_AFTER_POP + 0.0))*(time_into_auto_insertion-time_into_auto_insertion_offset)*line_dir_robot_auto_line_snapshot[i];
					VelDesired[i] = 0;
					AccelDesired[i] = 0;
				}
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 30) //[30] = "NEEDLExyz: Go far away from hand.";
			{
				PosDesired[1] = Pos_auto_line_backup_to_this_point_at_end[1] + ((Ymax_SliderManual - Pos_auto_line_backup_to_this_point_at_end[1])/auto_insertion_substate_time_vector[auto_insertion_substate])*(time_into_auto_insertion-time_into_auto_insertion_offset); //
				PosDesired[3] = Pos_auto_line_snapshot[3] - (Pos_auto_line_snapshot[3]/auto_insertion_substate_time_vector[auto_insertion_substate])*(time_into_auto_insertion-time_into_auto_insertion_offset); //Set yaw to 0 at end.
				PosDesired[4] = Pos_auto_line_snapshot[4] - (Pos_auto_line_snapshot[4]/auto_insertion_substate_time_vector[auto_insertion_substate])*(time_into_auto_insertion-time_into_auto_insertion_offset); //Set pitch to 0 at end.
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 31) //[31] = "ASSISTOBOT: Wait for human inspection while holding catheter securely.";
			{
				if(abs(assistoBot_ATI_force_vec[5]) <= assistoBot_ATI_human_signal_to_let_go_limit) //Using Tz (twist about Z).
				{
					//Do nothing.
				}
				else
				{
					auto_insertion_substate_time_vector[auto_insertion_substate] = time_into_auto_insertion-time_into_auto_insertion_offset; //Abort this state early.
				}
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else if(auto_insertion_substate == 32) //[32] = "ASSISTOBOT: Go far away from hand.";
			{
				auto_line_assistoBot_posDesired_vec[0] = -50; 
				auto_line_assistoBot_posDesired_vec[1] = -40; 
				auto_line_assistoBot_posDesired_vec[2] = 94; 
			}
			///////////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////////
			else
			{
				//we're going to let the program hang and see if this helps to avert the freakout in y-axis at the end
				//auto_is_actively_inserting_flag = 0;
				//auto_insertion_substate = 0; //WAS ORIGINALLY -1, BUT THIS MAY HAVE BEEN CAUSING THE VECTOR INDEXING ISSUES. CHANGING TO 0 MADE THEM DISAPPEAR.
			}
			///////////////////////////////////////////////////////////////////////////////////
		}
	}
}

void needleBot_controller::timer_update(QTimerEvent *event)
{
	///////////////////////////////////////////////////////////////////////////////////  In case we use the foot pedal to change the manual/haptic/auto control flags, we need to be able to change the radio buttons but from within the GUI thread.
	if(manual_control_button_needs_to_be_checked == 1)
	{
		manual_control_button->setChecked(manual_control_flag);
		manual_control_button_needs_to_be_checked = 0;
	}
	if(haptic_control_button_needs_to_be_checked == 1)
	{
		haptic_control_button->setChecked(haptic_control_flag);
		haptic_control_button_needs_to_be_checked = 0;
	}
	if(auto_control_button_needs_to_be_checked == 1)
	{
		auto_control_button->setChecked(auto_control_flag);
		auto_control_button_needs_to_be_checked = 0;
	}
	///////////////////////////////////////////////////////////////////////////////////

	manual_control_flag = manual_control_button->isChecked();
	haptic_control_flag = haptic_control_button->isChecked();
	auto_control_flag = auto_control_button->isChecked();

	///////////////////////////////////////////////////////////////////////////////////
	if(manual_control_flag == 1 && last_manual_control_flag == 0)
	{
		slider_box_manual_x->setVal(PosActual[0]);
		slider_box_manual_y->setVal(PosActual[1]);
		slider_box_manual_z->setVal(PosActual[2]);
		slider_box_manual_yaw->setVal(PosActual[3]);
		slider_box_manual_pitch->setVal(PosActual[4]);
		slider_box_manual_roll->setVal(PosActual[5]);
		slider_box_manual_cathins->setVal(PosActual[6]);

		for(int k = 0; k < 5; k++)
		{
			Pos_manualSliderInput[k] = PosActual[k];
		}
	}
	///////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////////
	if(haptic_control_flag == 1 && last_haptic_control_flag == 0)
	{
		take_rezero_omni_snapshot(); //WHEN WE FIRST START HAPTIC CONTROL, MAKE SURE THAT WE'RE PROPERLY ZEROED FIRST!!!
		what_needs_to_happen_to_foot_pedal_focus = 1;
	}
	else if(haptic_control_flag == 0 && last_haptic_control_flag == 1)
	{
		what_needs_to_happen_to_foot_pedal_focus = -1;
	}
	else
	{
		what_needs_to_happen_to_foot_pedal_focus = 0;
	}
	///////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////////
	if(auto_control_flag == 0)
	{
		auto_insertion_substate = 0;////WAS ORIGINALLY -1, BUT THIS MAY HAVE BEEN CAUSING THE VECTOR INDEXING ISSUES. CHANGING TO 0 MADE THEM DISAPPEAR.
		auto_line_snapshot_has_been_taken = 0;
	}

	if(auto_control_flag == 1 && last_auto_control_flag == 0)
	{
		take_auto_line_snapshot();
		what_needs_to_happen_to_data_logger_file = 1;
	}
	else if(auto_control_flag == 0 && last_auto_control_flag == 1)
	{
		what_needs_to_happen_to_data_logger_file = -1;
	}
	else
	{
		//what_needs_to_happen_to_data_logger_file = 0; //Do nothing special, just log the data.
	}
	///////////////////////////////////////////////////////////////////////////////////


	X_label->setText("Xact: " + QString::number(PosActual[0]) + "<BR>Xdes: " + QString::number(PosDesired[0]));
	Y_label->setText("Yact: " + QString::number(PosActual[1]) + "<BR>Ydes: " + QString::number(PosDesired[1]));
	Z_label->setText("Zact: " + QString::number(PosActual[2]) + "<BR>Zdes: " + QString::number(PosDesired[2]));
	Yaw_label->setText("YawAct: " + QString::number(PosActual[3]) + "<BR>YawDes: " + QString::number(PosDesired[3]));
	Pitch_label->setText("PitchAct: " + QString::number(PosActual[4]) + "<BR>PitchDes: " + QString::number(PosDesired[4]));
	Roll_label->setText("RollAct: " + QString::number(PosActual[5]) + "<BR>RollDes: " + QString::number(PosDesired[5]));
	CathIns_label->setText("CathInsAct: " + QString::number(PosActual[6]) + "<BR>CathInsDes: " + QString::number(PosDesired[6]));
	auto_line_direction_label->setText("Auto Line:<BR>Lx:" + QString::number(line_dir_robot[0]) + "<BR>Ly: " + QString::number(line_dir_robot[1]) + "<BR>Lz: " + QString::number(line_dir_robot[2]));

	insertion_site_label->setText("Insertion Site:<BR>Px:" + QString::number(Pos_assistoBot_desired_insertion_site[0]) + "<BR>Py: " + QString::number(Pos_assistoBot_desired_insertion_site[1]) + "<BR>Pz: " + QString::number(Pos_assistoBot_desired_insertion_site[2]));
	assistoBot_pos_Actual_label->setText("assisoBot Act:<BR>Lx:" + QString::number(auto_line_assistoBot_posActual_vec[0]) + "<BR>Ly: " + QString::number(auto_line_assistoBot_posActual_vec[1]) + "<BR>Lz: " + QString::number(auto_line_assistoBot_posActual_vec[2]));
	assistoBot_pos_Desired_label->setText("assisoBot Des:<BR>Lx:" + QString::number(auto_line_assistoBot_posDesired_vec[0]) + "<BR>Ly: " + QString::number(auto_line_assistoBot_posDesired_vec[1]) + "<BR>Lz: " + QString::number(auto_line_assistoBot_posDesired_vec[2]));
	needleBot_pos_EndPoint_label->setText("needleBot line END:<BR>Lx:" + QString::number(Pos_auto_line_endpoint[0]) + "<BR>Ly: " + QString::number(Pos_auto_line_endpoint[1]) + "<BR>Lz: " + QString::number(Pos_auto_line_endpoint[2]));

	auto_line_insertion_depth = auto_line_insertion_depth_spinbox->getValue();
	auto_line_insertion_time_limit = auto_line_insertion_time_limit_spinbox->getValue();
	auto_line_insertion_depth_AFTER_POP = auto_line_insertion_depth_AFTER_POP_spinbox->getValue();
	auto_line_insertion_time_limit_AFTER_POP = auto_line_insertion_time_limit_AFTER_POP_spinbox->getValue();
	auto_line_CATH_insertion_depth = auto_line_CATH_insertion_depth_spinbox->getValue();
	auto_line_CATH_insertion_time_limit = auto_line_CATH_insertion_time_limit_spinbox->getValue();
	assistoBot_hand_contact_force_Z_threshold= assistoBot_hand_contact_force_Z_threshold_spinbox->getValue();
	assistoBot_hand_contact_traction_force_threshold = assistoBot_hand_contact_traction_force_threshold_spinbox->getValue();

	QString state_string_description_to_display;
	double time_into_current_substate_action_to_display;
	if(auto_insertion_substate == -1 || auto_is_actively_inserting_flag == 0)
	{
		state_string_description_to_display = "Not started auto insertion yet.";
		time_into_current_substate_action_to_display = -1;
	}
	else
	{
		state_string_description_to_display = auto_insertion_substate_string_descriptor_vec[auto_insertion_substate];
		time_into_current_substate_action_to_display = auto_insertion_substate_time_vector[auto_insertion_substate];
	}

	auto_insertion_substate_label->setText("Substate: " + QString::number(auto_insertion_substate) + " , Des: " + state_string_description_to_display + "<BR>Time Total: " + QString::number(time_into_auto_insertion) + " , Substate Time: " + QString::number(time_into_auto_insertion-time_into_auto_insertion_offset) + " out of " + QString::number(time_into_current_substate_action_to_display) + " sec.");
	insertion_depth_label->setText("Ins Depth: " + QString::number(current_insertion_depth) + "<BR>Ins Depth at Pop: " + QString::number(insertion_depth_before_detecting_pop)); 
	debug_label->setText("Debug: " + QString::number(what_needs_to_happen_to_data_logger_file)); 

	QString assistoBot_hand_contact_force_Z_QSTRING_TO_DISPLAY = "Not exceeded.";
	if(assistoBot_hand_contact_force_Z_THRESHOLD_EXCEEDED_BOOL == 1)
	{
		assistoBot_hand_contact_force_Z_QSTRING_TO_DISPLAY = "Threshold exceeded.";
	}
	assistoBot_hand_contact_force_Z_label->setText("Fz Cur: " + QString::number(assistoBot_hand_contact_force_Z) + " ,<BR>" + assistoBot_hand_contact_force_Z_QSTRING_TO_DISPLAY);

	QString assistoBot_hand_contact_traction_force_QSTRING_TO_DISPLAY = "Not exceeded.";
	if(assistoBot_hand_contact_traction_force_THRESHOLD_EXCEEDED_BOOL == 1)
	{
		assistoBot_hand_contact_traction_force_QSTRING_TO_DISPLAY = "Threshold exceeded.";
	}
	assistoBot_hand_contact_traction_force_label->setText("Traction F Cur: " + QString::number(assistoBot_hand_contact_traction_force) + " ,<BR>" + assistoBot_hand_contact_traction_force_QSTRING_TO_DISPLAY);

	if(manual_control_flag == 0)
	{
		slider_box_manual_x->setVal(PosActual[0]);
		slider_box_manual_y->setVal(PosActual[1]);
		slider_box_manual_z->setVal(PosActual[2]);
		slider_box_manual_yaw->setVal(PosActual[3]);
		slider_box_manual_pitch->setVal(PosActual[4]);
		slider_box_manual_roll->setVal(PosActual[5]);
		slider_box_manual_cathins->setVal(PosActual[6]);

		for(int k = 0; k < 5; k++)
		{
			Pos_manualSliderInput[k] = PosActual[k];
		}
	}

	last_manual_control_flag = manual_control_flag;
	last_haptic_control_flag = haptic_control_flag;
	last_auto_control_flag = auto_control_flag;
}


void needleBot_controller::home(void)
{
    printf("Homeing... \n");

  }

bool needleBot_controller::isManualControl(void)
{
    return manual_control_flag;
}

void needleBot_controller::x_slider_changed(void)
{
    Pos_manualSliderInput[0] = slider_box_manual_x->getSliderVal();
    isDraggedFlag = true;
}

void needleBot_controller::y_slider_changed(void)
{
    Pos_manualSliderInput[1] = slider_box_manual_y->getSliderVal();
    isDraggedFlag = true;
}

void needleBot_controller::z_slider_changed(void)
{
    Pos_manualSliderInput[2] = slider_box_manual_z->getSliderVal();
    isDraggedFlag = true;
}

void needleBot_controller::yaw_slider_changed(void)
{
    Pos_manualSliderInput[3] = slider_box_manual_yaw->getSliderVal();
    isDraggedFlag = true;
}

void needleBot_controller::pitch_slider_changed(void)
{
    Pos_manualSliderInput[4] = slider_box_manual_pitch->getSliderVal();
    isDraggedFlag = true;
}

void needleBot_controller::roll_slider_changed(void)
{
    Pos_manualSliderInput[5] = slider_box_manual_roll->getSliderVal();
    isDraggedFlag = true;
}

void needleBot_controller::cathins_slider_changed(void)
{
    Pos_manualSliderInput[6] = slider_box_manual_cathins->getSliderVal();
    isDraggedFlag = true;
}

void needleBot_controller::take_rezero_omni_snapshot(void)
{
	Pos_omniInput_rezero_snapshot[0] = Pos_omniInput_raw[0] - PosActual[0]; //Should make the omni read exactly where the robot is.
	Pos_omniInput_rezero_snapshot[1] = Pos_omniInput_raw[1] - PosActual[1]; //Should make the omni read exactly where the robot is.
	Pos_omniInput_rezero_snapshot[2] = Pos_omniInput_raw[2] - PosActual[2]; //Should make the omni read exactly where the robot is.
	Pos_omniInput_rezero_snapshot[3] = Pos_omniInput_raw[3] - PosActual[3]; //Should make the omni read exactly where the robot is.
	Pos_omniInput_rezero_snapshot[4] = Pos_omniInput_raw[4] - PosActual[4]; //Should make the omni read exactly where the robot is.
	//Pos_omniInput_rezero_snapshot[5] = Pos_omniInput_raw[5];
	//Pos_omniInput_rezero_snapshot[6] = Pos_omniInput_raw[6];

	//cout << "Took another snapshot to rezero the needleBot Omni." << endl;
}

void needleBot_controller::take_auto_line_snapshot(void)
{
			////////////////////////////////////////////////////////////////////
			assistoBot_number_of_cleaning_horizontal_sweeps_N = 5.0;
			assistoBot_cleaning_horizontal_sweeps_total_time_allowed_T = 7.0;
			assistoBot_cleaning_horizontal_sweeps_amplitude_A = 8.0;
			assistoBot_z_drop_velocity = 10;
			assistoBot_Z_axis_force_limit = 0.5;
			assistoBot_traction_goal = 0.5;
			assistoBot_ATI_human_signal_to_let_go_limit = 0.1;
			auto_line_end_of_procedure_needle_retraction_distance = 53.0 + 5.0; //53mm gets completely out of catheter, +5mm gives us clearance.
			auto_line_needleBot_backs_up_at_start_to_give_room_distance = 75;
			////////////////////////////////////////////////////////////////////

	Pos_auto_line_snapshot[0] = PosActual[0];
	Pos_auto_line_snapshot[1] = PosActual[1];
	Pos_auto_line_snapshot[2] = PosActual[2];
	Pos_auto_line_snapshot[3] = PosActual[3];
	Pos_auto_line_snapshot[4] = PosActual[4];
	Pos_auto_line_snapshot[5] = PosActual[5];
	Pos_auto_line_snapshot[6] = 0;//PosActual[6]; //MAKE SURE THE CATH INS IS ALWAYS BACK

	Pos_assistoBot_auto_line_snapshot[0] = auto_line_assistoBot_posActual_vec[0];
	Pos_assistoBot_auto_line_snapshot[1] = auto_line_assistoBot_posActual_vec[1];
	Pos_assistoBot_auto_line_snapshot[2] = auto_line_assistoBot_posActual_vec[2];

	Pos_assistoBot_desired_insertion_site[0] = PosDesired[0]; //TELLS ASSISTOBOT WHERE THE ROBOT INTENDS TO INSERT INSTEAD OF WHERE IT CURRENTLY IS.
	Pos_assistoBot_desired_insertion_site[1] = PosDesired[1]; //TELLS ASSISTOBOT WHERE THE ROBOT INTENDS TO INSERT INSTEAD OF WHERE IT CURRENTLY IS.
	Pos_assistoBot_desired_insertion_site[2] = PosDesired[2]; //TELLS ASSISTOBOT WHERE THE ROBOT INTENDS TO INSERT INSTEAD OF WHERE IT CURRENTLY IS.

	cout << "Desired insertion site X: " << Pos_assistoBot_desired_insertion_site[0] << " Y: " << Pos_assistoBot_desired_insertion_site[1] << " Z "  << Pos_assistoBot_desired_insertion_site[2]  << endl; 

	line_dir_robot_auto_line_snapshot = line_dir_robot;  

	time_auto_line_snapshot = current_global_time;

	auto_insertion_substate = 0; //WAS ORIGINALLY -1, BUT THIS MAY HAVE BEEN CAUSING THE VECTOR INDEXING ISSUES. CHANGING TO 0 MADE THEM DISAPPEAR.


	
	//////////////////////////////////////// //MUST BE INITIALIZED TO THIS IN CASE WE MISS THE POP.
	for(int i = 0; i <= 2; i++)
	{
		Pos_auto_line_endpoint[i] = Pos_auto_line_snapshot[i] + auto_line_insertion_depth*line_dir_robot_auto_line_snapshot[i];
		Pos_auto_line_endpoint_AFTER_POP[i] = Pos_auto_line_snapshot[i] + (auto_line_insertion_depth + auto_line_insertion_depth_AFTER_POP)*line_dir_robot_auto_line_snapshot[i];
		Pos_auto_line_backup_to_this_point_at_end[i] = Pos_auto_line_endpoint_AFTER_POP[i] - (auto_line_insertion_depth + auto_line_insertion_depth_AFTER_POP + auto_line_end_of_procedure_needle_retraction_distance)*line_dir_robot_auto_line_snapshot[i];
		Pos_auto_line_needleBot_backs_up_at_start_to_give_room[i] = Pos_auto_line_snapshot[i] - auto_line_needleBot_backs_up_at_start_to_give_room_distance*line_dir_robot_auto_line_snapshot[i];
	}
	for(int j = 3; j <= 5; j++)
	{
		Pos_auto_line_endpoint[j] = Pos_auto_line_snapshot[j];
		Pos_auto_line_endpoint_AFTER_POP[j] = Pos_auto_line_snapshot[j];
		Pos_auto_line_backup_to_this_point_at_end[j] = Pos_auto_line_snapshot[j];
		Pos_auto_line_needleBot_backs_up_at_start_to_give_room[j]  = Pos_auto_line_snapshot[j];
	}
	Pos_auto_line_endpoint[6] = Pos_auto_line_snapshot[6] + auto_line_CATH_insertion_depth;

	insertion_depth_before_detecting_pop = auto_line_insertion_depth; 
	////////////////////////////////////////////////////////////////////////

	auto_is_actively_inserting_flag = 1;
	auto_line_snapshot_has_been_taken = 1;


			//////////////////////////////////////////////////////////////////////////////////////////////////////////////

			auto_insertion_substate_string_descriptor_vec[0] = "ASSISTOBOT: Move far away from the hand.";
			auto_insertion_substate_time_vector[0] = 5;

			auto_insertion_substate_string_descriptor_vec[1] = "ND: Close the cath swing arm.";
			auto_insertion_substate_time_vector[1] = 3;

			auto_insertion_substate_string_descriptor_vec[2] = "ND: Close the cath loop.";
			auto_insertion_substate_time_vector[2] = 3;

			auto_insertion_substate_string_descriptor_vec[3] = "NEEDLExyz: Back the needle far away to give assistoBot room.";
			auto_insertion_substate_time_vector[3] = 60.0; //ORIGINAL ROBOT CODE FOR CORRECT AUTO INSERTION IS 5.0 SECONDS!!!!

			auto_insertion_substate_string_descriptor_vec[4] = "ASSISTOBOT: Move XY a little to left of insertion site for IPA spraying.";
			auto_insertion_substate_time_vector[4] = 4;

			auto_insertion_substate_string_descriptor_vec[5] = "ASSISTOBOT: Move Z just above the insertion site.";
			auto_insertion_substate_time_vector[5] = 3;

			auto_insertion_substate_string_descriptor_vec[6] = "DYNAMIXEL: Close the IPA (spray it).";
			auto_insertion_substate_time_vector[6] = 0.5;

			auto_insertion_substate_string_descriptor_vec[7] = "DYNAMIXEL: Open the IPA (stop spraying it).";
			auto_insertion_substate_time_vector[7] = 0.25;

			auto_insertion_substate_string_descriptor_vec[8] = "ASSISTOBOT: Move XY right above insertion site.";
			auto_insertion_substate_time_vector[8] = 2;

			auto_insertion_substate_string_descriptor_vec[9] = "ASSISTOBOT: Drop Z until the force sensor reads above a certain level.";
			auto_insertion_substate_time_vector[9] = 10;

			auto_insertion_substate_string_descriptor_vec[10] = "ASSISTOBOT: Scrub insertion site.";
			auto_insertion_substate_time_vector[10] = assistoBot_cleaning_horizontal_sweeps_total_time_allowed_T;

			auto_insertion_substate_string_descriptor_vec[11] = "ASSISTOBOT: Move Z just above the insertion site.";
			auto_insertion_substate_time_vector[11] = 1;

			auto_insertion_substate_string_descriptor_vec[12] = "DYNAMIXEL: Close the air (spray it).";
			auto_insertion_substate_time_vector[12] = 1;

			auto_insertion_substate_string_descriptor_vec[13] = "DYNAMIXEL: Open the air (stop spraying it).";
			auto_insertion_substate_time_vector[13] = 0.25;

			auto_insertion_substate_string_descriptor_vec[14] = "ASSISTOBOT: Move to just behind the insertion site in preparation for applying traction.";
			auto_insertion_substate_time_vector[14] = 2;

			auto_insertion_substate_string_descriptor_vec[15] = "ASSISTOBOT: Drop Z until the force sensor reads above a certain level.";
			auto_insertion_substate_time_vector[15] = 2;

			auto_insertion_substate_string_descriptor_vec[16] = "ASSISTOBOT: Move backwards to apply traction.";
			auto_insertion_substate_time_vector[16] = 5;

			auto_insertion_substate_string_descriptor_vec[17] = "NEEDLExyz: Insert needle forward to original insertion point.";
			auto_insertion_substate_time_vector[17] = 5.0;

			auto_insertion_substate_string_descriptor_vec[18] = "NEEDLExyz: Wait for a little to let us know we're about to insert.";
			auto_insertion_substate_time_vector[18] = 1;

			auto_insertion_substate_string_descriptor_vec[19] = "NEEDLExyz: Insert needle forward along auto line until sensing the pop.";
			auto_insertion_substate_time_vector[19] = auto_line_insertion_time_limit;

			auto_insertion_substate_string_descriptor_vec[20] = "NEEDLExyz: Waiting time in between the pop and the further insertion into lumen.";
			auto_insertion_substate_time_vector[20] = 1.0;
			
			auto_insertion_substate_string_descriptor_vec[21] = "NEEDLExyz: Insert needle forward along auto line a little beyond pop to get cath into lumen.";
			auto_insertion_substate_time_vector[21] = auto_line_insertion_time_limit_AFTER_POP;

			auto_insertion_substate_string_descriptor_vec[22] = "NEEDLExyz: Insert the catheter forward.";
			auto_insertion_substate_time_vector[22] = auto_line_CATH_insertion_time_limit;

			auto_insertion_substate_string_descriptor_vec[23] = "ASSISTOBOT: Move Z up off of hand (release traction).";
			auto_insertion_substate_time_vector[23] = 2;

			auto_insertion_substate_string_descriptor_vec[24] = "ASSISTOBOT: Move XY to just behind the insertion site to prepare for clamping the cath.";
			auto_insertion_substate_time_vector[24] = 2;

			auto_insertion_substate_string_descriptor_vec[25] = "ASSISTOBOT: Drop Z until the force sensor reads above a certain level, clamp cath.";
			auto_insertion_substate_time_vector[25] = 2;

			auto_insertion_substate_string_descriptor_vec[26] = "ND: Open the cath loop.";
			auto_insertion_substate_time_vector[26] = 3.0;

			auto_insertion_substate_string_descriptor_vec[27] = "ND: Open the cath swing arm.";
			auto_insertion_substate_time_vector[27] = 3.0;

			auto_insertion_substate_string_descriptor_vec[28] = "ND: Retract the catheter backward.";
			auto_insertion_substate_time_vector[28] = auto_line_CATH_insertion_time_limit;

			auto_insertion_substate_string_descriptor_vec[29] = "NEEDLExyz: Retract the needle backward.";
			auto_insertion_substate_time_vector[29] = auto_line_insertion_time_limit + auto_line_insertion_time_limit_AFTER_POP;

			auto_insertion_substate_string_descriptor_vec[30] = "NEEDLExyz: Go far away from hand.";
			auto_insertion_substate_time_vector[30] = 5.0;

			auto_insertion_substate_string_descriptor_vec[31] = "ASSISTOBOT: Wait for human inspection while holding catheter securely.";
			auto_insertion_substate_time_vector[31] = 60;

			auto_insertion_substate_string_descriptor_vec[32] = "ASSISTOBOT: Go far away from hand.";
			auto_insertion_substate_time_vector[32] = 5;



			//////////////////////////////////////////////////////////////////////////////////////////////////////////////

	auto_line_assistoBot_posDesired_vec[0] = -50; //AS OUT OF THE WAY AS POSSIBLE//Pos_assistoBot_auto_line_snapshot[0]; //START OUT THE AUTO ROUTINE BY HAVING ASSISTOBOT HOLD ITS CURRENT POSITION AT THE SNAPSHOT COORDINATES
	auto_line_assistoBot_posDesired_vec[1] = -40; //AS OUT OF THE WAY AS POSSIBLE//Pos_assistoBot_auto_line_snapshot[1]; //START OUT THE AUTO ROUTINE BY HAVING ASSISTOBOT HOLD ITS CURRENT POSITION AT THE SNAPSHOT COORDINATES
	auto_line_assistoBot_posDesired_vec[2] = 94; //AS OUT OF THE WAY AS POSSIBLE //Pos_assistoBot_auto_line_snapshot[2]; //START OUT THE AUTO ROUTINE BY HAVING ASSISTOBOT HOLD ITS CURRENT POSITION AT THE SNAPSHOT COORDINATES

	cout << "Took another snapshot for the auto line controller." << endl;
}

void needleBot_controller::updateOmniInput(std::vector<double> omni_pos_vec_in)
{
	Pos_omniInput_raw = omni_pos_vec_in;
	
	for(int i = 0; i < Pos_omniInput_raw.size(); i++)
	{
		Pos_omniInput_offset[i] = Pos_omniInput_raw[i] - Pos_omniInput_rezero_snapshot[i];
	}
}

void needleBot_controller::calculate_robot_line(void)
{
	float R;
	R = sin(PosActual[4]*pi_define/180);

	p_line_robot.set(PosActual[0], PosActual[1], PosActual[2]);
	line_dir_robot.set(R*sin(PosActual[3]*pi_define/180), -R*cos(PosActual[3]*pi_define/180), -cos(PosActual[4]*pi_define/180));

	line_dir_robot.normalize();
	cml::matrix_rotation_euler<float>(rot_matrix, PosActual[5]*pi_define/180, PosActual[4]*pi_define/180, PosActual[3]*pi_define/180, cml::euler_order_zxz); //The function is tempalted,so we must put <float> to specify to use floats
}

void needleBot_controller::updateInsertionForcePopState(double insertion_force_pop_detection_state_latched_in)
{
	insertion_force_pop_detection_state_latched = insertion_force_pop_detection_state_latched_in;
}

bool needleBot_controller::shouldOutsideThreadsYieldControlToNeedleBotController(void)
{
	if(auto_control_flag == 1 && auto_is_actively_inserting_flag == 1 && auto_line_snapshot_has_been_taken == 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void needleBot_controller::updateAssistoBotForce(std::vector<double> assistoBot_ATI_force_vec_in)
{
	assistoBot_ATI_force_vec = assistoBot_ATI_force_vec_in;

	/////////////////////////// UPDATE VERTICAL CONTACT FORCE
	assistoBot_hand_contact_force_Z = abs(assistoBot_ATI_force_vec[2]);
	if(assistoBot_hand_contact_force_Z > assistoBot_hand_contact_force_Z_threshold)
	{
		assistoBot_hand_contact_force_Z_THRESHOLD_EXCEEDED_BOOL = 1;
	}
	else
	{
		assistoBot_hand_contact_force_Z_THRESHOLD_EXCEEDED_BOOL = 0;
	}
	///////////////////////////

	/////////////////////////// UPDATE TRACTION FORCE
	assistoBot_hand_contact_traction_force = abs(assistoBot_ATI_force_vec[0]);
	if(assistoBot_hand_contact_traction_force > assistoBot_hand_contact_traction_force_threshold)
	{
		assistoBot_hand_contact_traction_force_THRESHOLD_EXCEEDED_BOOL = 1;
	}
	else
	{
		assistoBot_hand_contact_traction_force_THRESHOLD_EXCEEDED_BOOL = 0;
	}
	///////////////////////////
}

void needleBot_controller::set_manual_control_flag(bool val)
{
	manual_control_flag = val;
	manual_control_button_needs_to_be_checked = 1;

	if(manual_control_flag == 1 && last_manual_control_flag == 0)
	{

		for(int k = 0; k < 5; k++)
		{
			Pos_manualSliderInput[k] = PosActual[k];
		}
	}

	last_manual_control_flag == !val; //JUST ADDED THIS 07/27/14 @ 8:43PM AND CNA'T TEST THE NEEDLEBOT RIGHT NOW (WORKING ON ASSISTOBOT). LAST NIGHT THERE WEREN'T ANY PROBLEMS WITH PERFORMANCE AND THE OLD CODE, BUT WILL NEED TO TEST THIS.
}

void needleBot_controller::set_haptic_control_flag(bool val)
{
	haptic_control_flag = val;
	haptic_control_button_needs_to_be_checked = 1;

	if(haptic_control_flag == 1 && last_haptic_control_flag == 0)
	{
		take_rezero_omni_snapshot(); //WHEN WE FIRST START HAPTIC CONTROL, MAKE SURE THAT WE'RE PROPERLY ZEROED FIRST!!!
		what_needs_to_happen_to_foot_pedal_focus = 1;
	}
	else if(haptic_control_flag == 0 && last_haptic_control_flag == 1)
	{
		what_needs_to_happen_to_foot_pedal_focus = -1;
	}
	else
	{
		what_needs_to_happen_to_foot_pedal_focus = 0;
	}


	last_haptic_control_flag == !val;
}

void needleBot_controller::set_auto_control_flag(bool val)
{
	auto_control_flag = val;
	auto_control_button_needs_to_be_checked = 1;
	last_auto_control_flag == !val;
}

