#include "print_motor_info.h"

int print_motor_width;
int print_motor_height;

print_motor_info::print_motor_info(QWidget *parent, int motor_number_in, QString motor_name_in, double Kp_start_val_in, double Kv_PIDcontroller_start_val_in, double Kv_velController_start_val_in, double Ki_start_val_in, double vel_filter_omega_start_val_in, double Kt_in, double motor_V_over_A_in, double voltage_for_max_cont_current_from_copely_in, double calibration_speed_for_starting_ZERO_on_home_sensor_in, double calibration_speed_for_starting_ONE_on_home_sensor_in, int control_mode_start_in, double position_offset_for_needleBot_to_assistoBot_calibration_in, double percent_effort_of_max_continuous_init_in):
    QWidget(parent)
{
		motor_number = motor_number_in;
		motor_name = motor_name_in;
		
		Kp_start_val = Kp_start_val_in;
		Kv_PIDcontroller_start_val = Kv_PIDcontroller_start_val_in;
		Kv_velController_start_val = Kv_velController_start_val_in;
		Ki_start_val = Ki_start_val_in;
		vel_filter_omega_start_val = vel_filter_omega_start_val_in;
		Kt = Kt_in;
		motor_V_over_A = motor_V_over_A_in;
		voltage_for_max_cont_current_from_copely = voltage_for_max_cont_current_from_copely_in;
		calibration_speed_for_starting_ZERO_on_home_sensor = calibration_speed_for_starting_ZERO_on_home_sensor_in;
		calibration_speed_for_starting_ONE_on_home_sensor = calibration_speed_for_starting_ONE_on_home_sensor_in;
		control_mode_start = control_mode_start_in;
		position_offset_for_needleBot_to_assistoBot_calibration = position_offset_for_needleBot_to_assistoBot_calibration_in;
		percent_effort_of_max_continuous_init = percent_effort_of_max_continuous_init_in;

		print_motor_width = 200;
        print_motor_height = 730;
		background_box = new QLabel("",this);
		background_box->setGeometry(0,0,print_motor_width,print_motor_height);
		background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(rgb(200,255,200))}");

		/////////////////////////background_box must be the first GUI object so that it doesn't cover over other objects!!!
		box_title = new QLabel(motor_name,this);
		box_title->setGeometry(10,10,150,30);
		box_title->setFont(QFont("Times", 12, QFont::Bold));
		box_title->setText(motor_name + " Motor, " + QString::number(motor_number));

		int labels_start_x = 10;
		int labels_start_y = 40;
		int labels_start_width = 150;
		int labels_start_height_single_line_text = 20;
		int labels_start_height_double_line_text = 35;
		int labels_start_height_inc = 1;

	    motor_enable_in_software_button = new QRadioButton("Enable Motor in Software", this);
	    motor_enable_in_software_button->setGeometry(QRect(labels_start_x, labels_start_y + 0*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_single_line_text));
	    motor_enable_in_software_button->setAutoExclusive(0);
	    motor_enable_in_software_button->setChecked(1);

		motor_enable_hardware_flag_label = new QLabel("", this);
		motor_enable_hardware_flag_label->setGeometry(QRect(labels_start_x, labels_start_y + 1*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_single_line_text));

		calibration_performed_flag_label = new QLabel("", this);
		calibration_performed_flag_label->setGeometry(QRect(labels_start_x, labels_start_y + 2*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_single_line_text));

		home_flag_label = new QLabel("", this);
		home_flag_label->setGeometry(QRect(labels_start_x, labels_start_y + 3*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_single_line_text));

		mag_flag_label = new QLabel("", this);
		mag_flag_label->setGeometry(QRect(labels_start_x, labels_start_y + 4*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_single_line_text));

		slot_flag_label = new QLabel("", this);
		slot_flag_label->setGeometry(QRect(labels_start_x, labels_start_y + 5*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_single_line_text));

		position_label = new QLabel("", this);
		position_label->setGeometry(QRect(labels_start_x, labels_start_y + 6*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_double_line_text));

		velocity_label = new QLabel("", this);
		velocity_label->setGeometry(QRect(labels_start_x, labels_start_y + 8*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_double_line_text));

		acceleration_label = new QLabel("", this);
		acceleration_label->setGeometry(QRect(labels_start_x, labels_start_y + 10*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_double_line_text));

		error_label = new QLabel("", this);
		error_label->setGeometry(QRect(labels_start_x, labels_start_y + 12*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_single_line_text));

		errorD_label = new QLabel("", this);
		errorD_label->setGeometry(QRect(labels_start_x, labels_start_y + 13*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_single_line_text));

		errorSum_label = new QLabel("", this);
		errorSum_label->setGeometry(QRect(labels_start_x, labels_start_y + 14*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_single_line_text));

		percent_effort_of_max_continuous = new QLabel("", this);
		percent_effort_of_max_continuous->setGeometry(QRect(labels_start_x, labels_start_y + 15*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_single_line_text));

		voltage_to_write_raw_label = new QLabel("", this);
		voltage_to_write_raw_label->setGeometry(QRect(labels_start_x, labels_start_y + 16*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_single_line_text));

		voltage_to_write_limited_label = new QLabel("", this);
		voltage_to_write_limited_label->setGeometry(QRect(labels_start_x, labels_start_y + 17*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_single_line_text));

		vel_filter_lambda_label = new QLabel("", this);
		vel_filter_lambda_label->setGeometry(QRect(labels_start_x, labels_start_y + 18*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_single_line_text));

		vel_calc_deltaT_and_F_label = new QLabel("", this);
		vel_calc_deltaT_and_F_label->setGeometry(QRect(labels_start_x, labels_start_y + 19*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_double_line_text));

		int spinbox_start_x = 5;
		int spinbox_start_y = labels_start_y + 21*(labels_start_height_single_line_text + labels_start_height_inc);
		int spinbox_start_width = 175;
		int spinbox_start_height = 30;
		int spinbox_start_height_inc = 2;

		Kp_spinbox = new print_labeled_spinbox(this, "Kp: ", 0.0001, 4, 0.0, 1000.0, Kp_start_val);
		Kp_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 0*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));

		Kv_PIDcontroller_spinbox = new print_labeled_spinbox(this, "Kv_PID: ", 0.0001, 4, 0.0, 1000.0, Kv_PIDcontroller_start_val);
		Kv_PIDcontroller_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 1*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));

		Kv_velController_spinbox = new print_labeled_spinbox(this, "Kv_Vel: ", 0.0001, 4, 0.0, 1000.0, Kv_velController_start_val);
		Kv_velController_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 2*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));

		Ki_spinbox = new print_labeled_spinbox(this, "Ki: ", 0.0001, 4, 0.0, 1000.0, Ki_start_val);
		Ki_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 3*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));

		max_allowable_current_percentage_spinbox = new print_labeled_spinbox(this, "Max Eff % of (" + QString::number(voltage_for_max_cont_current_from_copely, 'g', 3) + "v): ", 0.001, 3, 0.0,40.0, percent_effort_of_max_continuous_init);
		max_allowable_current_percentage_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 4*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));

		vel_filter_omega_spinbox = new print_labeled_spinbox(this, "Vel Omega: ", 0.001, 3, 0.0, 10000.0, vel_filter_omega_start_val);
		vel_filter_omega_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 5*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));

		control_mode_spinbox = new print_labeled_spinbox(this, "Control Mode: ", 1, 1, 0.0, 2, control_mode_start);
		control_mode_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 6*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));

		enable_motor_constants_spinbox_change_button = new QRadioButton("Enable Constants Change", this);
	    enable_motor_constants_spinbox_change_button->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 7*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));
	    enable_motor_constants_spinbox_change_button->setAutoExclusive(0);
	    enable_motor_constants_spinbox_change_button->setChecked(0);
		enable_motor_constants_spinbox_change_button->setEnabled(1);
		enable_motor_constants_spinbox_flag = 0;
	
		motor_enable_hardware_flag = 0;
		last_motor_enable_hardware_flag = 1;
		motor_enable_software_flag = 1;
		last_motor_enable_software_flag = 0;
		enable_motor_constants_spinbox_flag = 0;
		last_enable_motor_constants_spinbox_flag = 1;

		position_actual = 0;
		velocity_actual = 0;
		acceleration_actual  = 0;
		position_desired = 0;
		velocity_desired = 0;
		acceleration_desired = 0;
		error  = 0;
		errorD = 0;
		errorSum = 0;
		torque_to_write = 0;
		current_to_write = 0;
		voltage_to_write_raw  = 0;
		voltage_to_write_limited = 0;
		calibration_performed_flag = -1; //-1 = hasn't yet started calibration, 0 = calibrating, 1 = already calibrated
		home_flag_calibration_starting_value = -1;
		home_flag = -2;
		neg_slot_flag = -1;
		neg_mag_flag = -1;
		pos_slot_flag = -1;
		pos_mag_flag = -1;
}


void print_motor_info::timer_update(QTimerEvent *event)
{
	motor_enable_software_flag = motor_enable_in_software_button->isChecked();
	enable_motor_constants_spinbox_flag = enable_motor_constants_spinbox_change_button->isChecked();

	Kp = Kp_spinbox->getValue();
	Kv_PIDcontroller = Kv_PIDcontroller_spinbox->getValue();
	Kv_velController = Kv_velController_spinbox->getValue();
	Ki = Ki_spinbox->getValue();
	max_allowable_current_percentage = max_allowable_current_percentage_spinbox->getValue();
	vel_filter_omega = vel_filter_omega_spinbox->getValue();
	control_mode = control_mode_spinbox->getValue();

	//cout << "last: " << last_motor_enable_software_flag << " current: " << motor_enable_software_flag << endl;
	if((motor_enable_software_flag != last_motor_enable_software_flag) || (motor_enable_hardware_flag != last_motor_enable_hardware_flag))
	{
		
		if(motor_enable_hardware_flag == 0 || motor_enable_software_flag == 0)
		{
			disable_in_software();
		}
		else if(motor_enable_hardware_flag == 1 && motor_enable_software_flag == 1)
		{
			enable_in_software();
		}
	}

	if(enable_motor_constants_spinbox_flag != last_enable_motor_constants_spinbox_flag)
	{
		setEnableStateAllSpinboxes(enable_motor_constants_spinbox_flag);
	}

	motor_enable_hardware_flag_label->setText("Hardware Enable: " + QString::number(motor_enable_hardware_flag));
	position_label->setText("Pos Act: " + QString::number(position_actual) + "<BR>Pos des: " + QString::number(position_desired));
	velocity_label->setText("Vel Act: " + QString::number(velocity_actual) + "<BR>Vel des: " + QString::number(velocity_desired));
	acceleration_label->setText("Accel Act: " + QString::number(acceleration_actual) + "<BR>Accel des: " + QString::number(velocity_desired));
	error_label->setText("Error: " + QString::number(error));
	errorD_label->setText("ErrorD: " + QString::number(errorD));
	errorSum_label->setText("ErrorSum: " + QString::number(errorSum));
	percent_effort_of_max_continuous->setText("%Effort max cont: %" + QString::number(percent_effort_max_continous));
	voltage_to_write_raw_label->setText("Volt Out Raw: " + QString::number(voltage_to_write_raw));
	voltage_to_write_limited_label->setText("Volt Out Limited: " + QString::number(voltage_to_write_limited));
	vel_filter_lambda_label->setText("Vel Filter Lambda: " + QString::number(vel_filter_lambda));
	vel_calc_deltaT_and_F_label->setText("Vel Loop T: " + QString::number(loop_deltaT) + "<BR>Vel Loop F: " + QString::number(loop_F));

	calibration_performed_flag_label->setText("Cal State: " + QString::number(calibration_performed_flag));
	home_flag_label->setText("Home Start: " + QString::number(home_flag_calibration_starting_value) + " Cur Home: " + QString::number(home_flag));
	mag_flag_label->setText("Mag  Sensors:   neg: " + QString::number(neg_mag_flag) + " pos: " + QString::number(pos_mag_flag));
	slot_flag_label->setText("Slot Sensors:   neg: " + QString::number(neg_slot_flag) + " pos: " + QString::number(pos_slot_flag));

	last_motor_enable_software_flag = motor_enable_software_flag;
	last_motor_enable_hardware_flag = motor_enable_hardware_flag;
	last_enable_motor_constants_spinbox_flag = enable_motor_constants_spinbox_flag;
}

void print_motor_info::enable_in_software()
{
	enable_motor_constants_spinbox_change_button->setEnabled(1);
	background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(200,255,200)}");
}

void print_motor_info::disable_in_software()
{
	setEnableStateAllSpinboxes(0);
	//enable_motor_constants_spinbox_change_button->setEnabled(0); //COMMENT OUT IF YOU WANT TO BE ABLE TO CHANGE SLIDERBOX VALUES DURING DISABLED STATE
	//enable_motor_constants_spinbox_change_button->setChecked(0); //COMMENT OUT IF YOU WANT TO BE ABLE TO CHANGE SLIDERBOX VALUES DURING DISABLED STATE
	background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(255,200,200)}");
}

void print_motor_info::setEnableStateAllSpinboxes(int val)
{
    if(val == 1)
	{
		Kp_spinbox->enable();
		Kv_PIDcontroller_spinbox->enable();
		Kv_velController_spinbox->enable();
		Ki_spinbox->enable();
		max_allowable_current_percentage_spinbox->enable();
		vel_filter_omega_spinbox->enable();
		control_mode_spinbox->enable();
	}
    else if(val == 0)
	{
		Kp_spinbox->disable();
		Kv_PIDcontroller_spinbox->disable();
		Kv_velController_spinbox->disable();
		Ki_spinbox->disable();
		max_allowable_current_percentage_spinbox->disable();
		vel_filter_omega_spinbox->disable();
		control_mode_spinbox->disable();
	}
}

void print_motor_info::update_position_and_velocity(double pos_in, double vel_in)
{
	position_actual = pos_in;
	velocity_actual = vel_in;
}

void print_motor_info::compute_errors_and_voltage(void)
{
	error = position_desired - position_actual;
	errorD = velocity_desired - velocity_actual;

	double v_max, v, x_d_dot, arg;

	double ki_threshold = 1;
	double errorSum_max = 200;
	double errorSum_temp;

	//if(abs(errorSum) <= ki_threshold)
	//{
		errorSum_temp = errorSum + error; 
		errorSum = voltage_saturation(errorSum_temp, errorSum_max, -errorSum_max);
	//}

	if(control_mode == 0) //Pure velocity controller
	{
		torque_to_write = Kv_velController*errorD;
	}
	else if(control_mode == 1) //PID controller
	{
		torque_to_write = Kp*error + Kv_PIDcontroller*errorD + Ki*errorSum;
	}
	else if(control_mode == 2) //saturated PID controller that switches to velocity control past a certain limit.
	{
	/*	x_d_dot = (Kp/Kv_velController)*error + 0.00001;
		v_max = 5;
		arg = v_max/abs(x_d_dot);
		if(abs(arg) <= 1)
		{
			v = arg; //vel control
		}
		else if(abs(arg) > 1)
		{
			v = arg/abs(arg); //position control
		}

		torque_to_write = -Kv_velController*(errorD - v*x_d_dot);
		*/

	/*	double error_max = 1;
		v_max = 10;
		double error_sign;	
		if(abs(error) >= error_max)
		{
			if(error > 0)
			{
				velocity_desired = v_max;
			}
			else
			{
				velocity_desired = -1.0*v_max;
			}
			
			errorD = velocity_desired - velocity_actual;
			torque_to_write = Kv_velController*errorD;
		}
		else
		{
			velocity_desired = 0;
			torque_to_write = Kp*error + Kv_PIDcontroller*errorD + Ki*errorSum;
		}
	*/
	
		double error_max = 1.0;
		if(abs(error) > error_max)
		{
			if(error > 0)
			{
				error = error_max;
			}
			else
			{
				error = -1.0*error_max;
			}
		}
		torque_to_write = Kp*error + Kv_PIDcontroller*errorD;

	}

	current_to_write = torque_to_write/Kt;  //Kt has units of Nmm/Apk
	voltage_to_write_raw = current_to_write*motor_V_over_A;

	voltage_for_max_allowable_current = max_allowable_current_percentage*voltage_for_max_cont_current_from_copely;
	voltage_to_write_limited = voltage_saturation(voltage_to_write_raw, voltage_for_max_allowable_current, -1*voltage_for_max_allowable_current);

	percent_effort_max_continous = 100.0*voltage_to_write_raw/voltage_for_max_cont_current_from_copely;
}

double print_motor_info::voltage_saturation(double val_in, double max_V, double min_V)
{
	double desired_V = val_in;

    if(desired_V > max_V)
    {
        desired_V = max_V;
    }
    else if(desired_V < min_V)
    {
		desired_V = min_V;
    }

    return desired_V;
}

 void print_motor_info::update_home_and_limit_flags(int home_flag_in, int neg_slot_flag_in, int neg_mag_flag_in, int pos_slot_flag_in, int pos_mag_flag_in)
 {
	home_flag = home_flag_in;
	neg_slot_flag = neg_slot_flag_in;
	neg_mag_flag = neg_mag_flag_in;
	pos_slot_flag = pos_slot_flag_in;
	pos_mag_flag = pos_mag_flag_in;
 }

 void print_motor_info::setControlMode(int val)
 {
	 if(val != control_mode)
	 {
		 control_mode = val;
		 control_mode_spinbox->setValue(val);
	 }
 }
