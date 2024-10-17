#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <qwt_slider.h>
#include <QBoxLayout>
#include <iostream>

#define pi_define 3.141592654

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
		mainWindow_gui_single_monitor_width = 2560;
		mainWindow_gui_single_monitor_height = 2*1440; 
		mainWindow_gui_start_x_top_monitor = 10;
		mainWindow_gui_start_y_top_monitor = -1440;
		mainWindow_gui_start_x_bottom_monitor = mainWindow_gui_start_x_top_monitor;
		mainWindow_gui_start_y_bottom_monitor = 1440;

		///////////////////////////////////////////////////////
        end_program_button = new QPushButton(tr("End Program"), this);
        end_program_button->setFont(QFont("Times", 12, QFont::Bold));
        end_program_button->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 4px;background-color: rgb(98,246,135)}");
        end_program_button->setGeometry(QRect(5, mainWindow_gui_start_y_bottom_monitor + 35, 120, 20));
        connect(end_program_button, SIGNAL(clicked()), this, SLOT(end_program()));
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////
		double servo_loop_frequency_max = 2500;
		servo_loop_frequency = 2000;
		servo_loop_frequency_slider = new slider_with_box(this, "Servo Loop Frequency", 0, 100, servo_loop_frequency_max, 1, servo_loop_frequency);
		servo_loop_frequency_slider->setGeometry(QRect(140, mainWindow_gui_start_y_bottom_monitor + 5, 450, 60));
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////
		mainwindow_current_global_time_label = new QLabel("",this);
		mainwindow_current_global_time_label->setGeometry(QRect(5, mainWindow_gui_start_y_bottom_monitor + 100, 200, 30));
		mainwindow_current_global_time_label->setFont(QFont("Times", 10));
		mainwindow_current_global_time_label->setStyleSheet("*{color:black; border: 2px solid rgb(200,200,255); border-radius: 5px; background-color: rgb(200,200,255)}");
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////
		mainwindow_debug_label = new QLabel("",this);
		mainwindow_debug_label->setGeometry(QRect(235, mainWindow_gui_start_y_bottom_monitor + 100, 200, 30));
		mainwindow_debug_label->setFont(QFont("Times", 10));
		mainwindow_debug_label->setStyleSheet("*{color:black; border: 2px solid rgb(200,200,255); border-radius: 5px; background-color: rgb(200,200,255)}");
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////
		data_log = new my_data_log(this);
		data_log->setGeometry(QRect(600, mainWindow_gui_start_y_bottom_monitor + 5, 380, 150));
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////
        needle_driver_programmer = new my_needle_driver_programmer(this, "Needle Driver programmer");
        needle_driver_programmer->setGeometry(QRect(5, mainWindow_gui_start_y_bottom_monitor + 150,400,1250));
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////
		M_HAP_ARM = new HAP_ARM(this);
        M_HAP_ARM->setGeometry(QRect(2560-855, mainWindow_gui_start_y_bottom_monitor + 10 , 835, 355));
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////
        stepper_Rx_mutex = new QMutex();

		int stepper_start_x = 415;
		int stepper_start_y = mainWindow_gui_start_y_bottom_monitor + 10 + 355 + 5 + 280 - 490; // mainWindow_gui_start_y_bottom_monitor + 135; //
		int stepper_width = 400;
		int stepper_width_inc = 10;
		int stepper_height = 490;

		int stepper_programmer_start_x = 2560 - 510 - 20;
		int stepper_programmer_start_y = mainWindow_gui_start_y_bottom_monitor + 10 + 355 + 5;
		int stepper_programmer_width = 510;
		int stepper_programmer_height = 340;
		stepper_programmer = new my_stepper_programmer(this, "Stepper programmer");
		stepper_programmer->setGeometry(QRect(stepper_programmer_start_x, stepper_programmer_start_y, stepper_programmer_width, stepper_programmer_height));
       
        float stepper_0_pos_min = 0;
        float stepper_0_pos_max = (-1.25 + 96.674)*(2*pi_define/10.0); //converts mm to rad
        float stepper_0_pos_inc = 0.9*pi_define/180.0;
        float stepper_0_pos_init = stepper_0_pos_max;
        float stepper_0_vel_min = 1;
        float stepper_0_vel_max = 100;
        float stepper_0_vel_inc = 1;
        float stepper_0_vel_init = 2;
        float stepper_0_home_speed_min = 4;
        float stepper_0_home_speed_max = 100;
        float stepper_0_home_speed_inc = 1;
        float stepper_0_home_speed_init = stepper_0_home_speed_min;

        stepper_0 = new my_stepper(this, "Stepper 0", 0, stepper_0_pos_min, stepper_0_pos_max, stepper_0_pos_inc, stepper_0_pos_init, stepper_0_vel_min, stepper_0_vel_max, stepper_0_vel_inc, stepper_0_vel_init, stepper_0_home_speed_min, stepper_0_home_speed_max, stepper_0_home_speed_inc, stepper_0_home_speed_init, 225, 500, stepper_programmer);
        stepper_0->setGeometry(QRect(stepper_start_x + 0*(stepper_width + stepper_width_inc), stepper_start_y, stepper_width, stepper_height));

        float stepper_1_pos_min = (45.16-53.53)*pi_define/180.0;
        float stepper_1_pos_max = (45.16+50.40)*pi_define/180.0;
        float stepper_1_pos_inc = (1.8*pi_define/180.0)*(5.0/85.0);
        float stepper_1_pos_init = 45.16*pi_define/180.0;
        float stepper_1_vel_min = 1;
        float stepper_1_vel_max = 100;
        float stepper_1_vel_inc = 1;
        float stepper_1_vel_init = 10;
        float stepper_1_home_speed_min = 3;
        float stepper_1_home_speed_max = 100;
        float stepper_1_home_speed_inc = 1;
        float stepper_1_home_speed_init = stepper_1_home_speed_min;

        stepper_1 = new my_stepper(this, "Stepper 1", 1, stepper_1_pos_min, stepper_1_pos_max, stepper_1_pos_inc, stepper_1_pos_init, stepper_1_vel_min, stepper_1_vel_max, stepper_1_vel_inc, stepper_1_vel_init, stepper_1_home_speed_min, stepper_1_home_speed_max, stepper_1_home_speed_inc, stepper_1_home_speed_init, 364, 300, stepper_programmer);
        stepper_1->setGeometry(QRect(stepper_start_x + 1*(stepper_width + stepper_width_inc), stepper_start_y, stepper_width, stepper_height));

        float stepper_2_pos_min = (151.82-63.73)*pi_define/180.0;
        float stepper_2_pos_max = (151.82+62.65)*pi_define/180.0;
        float stepper_2_pos_inc = (1.8*pi_define/180.0)*(5.0/85.0);
        float stepper_2_pos_init = 151.82*pi_define/180.0;
        float stepper_2_vel_min = 1;
        float stepper_2_vel_max = 100;
        float stepper_2_vel_inc = 1;
        float stepper_2_vel_init = 10;
        float stepper_2_home_speed_min = 3;
        float stepper_2_home_speed_max = 100;
        float stepper_2_home_speed_inc = 1;
        float stepper_2_home_speed_init = stepper_2_home_speed_min;

        stepper_2 = new my_stepper(this, "Stepper 2", 3, stepper_2_pos_min, stepper_2_pos_max, stepper_2_pos_inc, stepper_2_pos_init, stepper_2_vel_min, stepper_2_vel_max, stepper_2_vel_inc, stepper_2_vel_init, stepper_2_home_speed_min, stepper_2_home_speed_max, stepper_2_home_speed_inc, stepper_2_home_speed_init, 162, 768, stepper_programmer);
        stepper_2->setGeometry(QRect(stepper_start_x + 2*(stepper_width + stepper_width_inc), stepper_start_y, stepper_width, stepper_height));
	
		float BP_pressure_init = 10;
		my_BP_cuff = new BP_cuff(this, 0, 1023, 1, BP_pressure_init, stepper_programmer);
        my_BP_cuff->setGeometry(QRect(stepper_start_x + 3*(stepper_width + stepper_width_inc), mainWindow_gui_start_y_bottom_monitor + 10 + 355 + 5, 380, 280));
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////
		my_DAQ = new DAQ_board(this);
		my_DAQ->setGeometry(QRect(1*mainWindow_gui_single_monitor_width - mainWindow_gui_start_x_top_monitor -250-20, 10, 250, mainWindow_gui_single_monitor_height-mainWindow_gui_start_y_top_monitor-30));
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////
		int ATI_start_x = 5;
		int ATI_start_y = 395;
		int ATI_width = 1280;
		int ATI_height = 380;
		int ATI_height_inc = 10;

		ATI_assistoBot = new ATI_force_sensor(this, "C:/Program Files/ATI Industrial Automation/ATIDAQFT.NET/assistoBot_FT6796.cal", "AssistoBot ATI-Mini-40");  
		ATI_assistoBot->setGeometry(QRect(ATI_start_x, ATI_start_y + 0*(ATI_height + ATI_height_inc), ATI_width, ATI_height));

		ATI_needleBot = new ATI_force_sensor(this, "C:/Program Files/ATI Industrial Automation/ATIDAQFT.NET/needleBot_FT8901.cal", "NeedleBot ATI-Mini-40");
        ATI_needleBot->setGeometry(QRect(ATI_start_x, ATI_start_y + 1*(ATI_height + ATI_height_inc), ATI_width, ATI_height));
		///////////////////////////////////////////////////////


		///////////////////////////////////////////////////////
		int motor_GUI_vec_start_x = 410;
		int motor_GUI_vec_start_y = mainWindow_gui_start_y_bottom_monitor + 1440 - 780;
		int motor_GUI_vec_height = 730;
		int motor_GUI_vec_width = 200;
		int motor_GUI_vec_width_inc = 5;

		QString motor_name_array[7] = {"X", "Y", "Z", "Yaw", "Pitch", "Roll", "Cath Ins"};

		double motor_Kp_array[7] = {rd_Kp_start_val_0, rd_Kp_start_val_1, rd_Kp_start_val_2, rd_Kp_start_val_3, rd_Kp_start_val_4, rd_Kp_start_val_5, rd_Kp_start_val_6};
		double motor_Kv_PIDcontroller_array[7] = {rd_Kv_PIDcontroller_start_val_0, rd_Kv_PIDcontroller_start_val_1, rd_Kv_PIDcontroller_start_val_2, rd_Kv_PIDcontroller_start_val_3, rd_Kv_PIDcontroller_start_val_4, rd_Kv_PIDcontroller_start_val_5, rd_Kv_PIDcontroller_start_val_6};
		double motor_Kv_velController_array[7] = {rd_Kv_velController_start_val_0, rd_Kv_velController_start_val_1, rd_Kv_velController_start_val_2, rd_Kv_velController_start_val_3, rd_Kv_velController_start_val_4, rd_Kv_velController_start_val_5, rd_Kv_velController_start_val_6};
			
		double motor_Ki_array[7] = {rd_Ki_start_val_0, rd_Ki_start_val_1, rd_Ki_start_val_2, rd_Ki_start_val_3, rd_Ki_start_val_4, rd_Ki_start_val_5, rd_Ki_start_val_6};
		double vel_filter_omega_array[7] = {rd_vel_filter_omega_0, rd_vel_filter_omega_1, rd_vel_filter_omega_2, rd_vel_filter_omega_3, rd_vel_filter_omega_4, rd_vel_filter_omega_5, rd_vel_filter_omega_6};
		double motor_Kt_array[7] = {rd_Kt_0, rd_Kt_1, rd_Kt_2, rd_Kt_3, rd_Kt_4, rd_Kt_5, rd_Kt_6};
		double motor_V_over_A[7] = {rd_motor_V_over_A_0, rd_motor_V_over_A_1, rd_motor_V_over_A_2, rd_motor_V_over_A_3, rd_motor_V_over_A_4, rd_motor_V_over_A_5, rd_motor_V_over_A_6};
		double voltage_for_max_cont_current_from_copely_array[7] = {rd_voltage_for_max_cont_current_from_copely_0, rd_voltage_for_max_cont_current_from_copely_1, rd_voltage_for_max_cont_current_from_copely_2, rd_voltage_for_max_cont_current_from_copely_3, rd_voltage_for_max_cont_current_from_copely_4, rd_voltage_for_max_cont_current_from_copely_5, rd_voltage_for_max_cont_current_from_copely_6};
		double calibration_speed_for_starting_ZERO_on_home_sensor_array[7] = {rd_calibration_speed_for_starting_ZERO_on_home_sensor_0, rd_calibration_speed_for_starting_ZERO_on_home_sensor_1, rd_calibration_speed_for_starting_ZERO_on_home_sensor_2, rd_calibration_speed_for_starting_ZERO_on_home_sensor_3, rd_calibration_speed_for_starting_ZERO_on_home_sensor_4, rd_calibration_speed_for_starting_ZERO_on_home_sensor_5, rd_calibration_speed_for_starting_ZERO_on_home_sensor_6};
		double calibration_speed_for_starting_ONE_on_home_sensor_array[7] = {rd_calibration_speed_for_starting_ONE_on_home_sensor_0, rd_calibration_speed_for_starting_ONE_on_home_sensor_1, rd_calibration_speed_for_starting_ONE_on_home_sensor_2, rd_calibration_speed_for_starting_ONE_on_home_sensor_3, rd_calibration_speed_for_starting_ONE_on_home_sensor_4, rd_calibration_speed_for_starting_ONE_on_home_sensor_5, rd_calibration_speed_for_starting_ONE_on_home_sensor_6};
		double control_mode_start_array[7] = {rd_control_mode_start_0, rd_control_mode_start_1, rd_control_mode_start_2, rd_control_mode_start_3, rd_control_mode_start_4, rd_control_mode_start_5, rd_control_mode_start_6};
		double position_offset_for_needleBot_to_assistoBot_calibration_array[7] = {rd_position_offset_for_needleBot_to_assistoBot_calibration_0, rd_position_offset_for_needleBot_to_assistoBot_calibration_1, rd_position_offset_for_needleBot_to_assistoBot_calibration_2, rd_position_offset_for_needleBot_to_assistoBot_calibration_3, rd_position_offset_for_needleBot_to_assistoBot_calibration_4, rd_position_offset_for_needleBot_to_assistoBot_calibration_5, rd_position_offset_for_needleBot_to_assistoBot_calibration_6};
		double percent_effort_of_max_continuous_init_array[7] = {rd_percent_effort_of_max_continuous_init_0, rd_percent_effort_of_max_continuous_init_1, rd_percent_effort_of_max_continuous_init_2, rd_percent_effort_of_max_continuous_init_3, rd_percent_effort_of_max_continuous_init_4, rd_percent_effort_of_max_continuous_init_5, rd_percent_effort_of_max_continuous_init_6};

		for(int motor_num_index = 0; motor_num_index < 7; motor_num_index++)
		{
			motor_GUI_vec.push_back(new print_motor_info(this, motor_num_index, motor_name_array[motor_num_index], motor_Kp_array[motor_num_index], motor_Kv_PIDcontroller_array[motor_num_index], motor_Kv_velController_array[motor_num_index], motor_Ki_array[motor_num_index], vel_filter_omega_array[motor_num_index], motor_Kt_array[motor_num_index], motor_V_over_A[motor_num_index], voltage_for_max_cont_current_from_copely_array[motor_num_index], calibration_speed_for_starting_ZERO_on_home_sensor_array[motor_num_index], calibration_speed_for_starting_ONE_on_home_sensor_array[motor_num_index], control_mode_start_array[motor_num_index], position_offset_for_needleBot_to_assistoBot_calibration_array[motor_num_index], percent_effort_of_max_continuous_init_array[motor_num_index]));
			motor_GUI_vec[motor_num_index]->setGeometry(QRect(motor_GUI_vec_start_x + motor_num_index*(motor_GUI_vec_width + motor_GUI_vec_width_inc), motor_GUI_vec_start_y, motor_GUI_vec_width, motor_GUI_vec_height));
		}
		///////////////////////////////////////////////////////


		///////////////////////////////////////////////////////
		needleBot_controller_instance = new needleBot_controller(this, "NeedleBot Controller");
        needleBot_controller_instance->setGeometry(QRect(2560-710, mainWindow_gui_start_y_bottom_monitor + 1440 - 725, 690, 675));
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////
		int dynamixel_servo_start_x = 310;
		int dynamixel_servo_start_y = 5;
		int dynamixel_servo_width = 385;
		int dynamixel_servo_width_inc = 5;
		int dynamixel_servo_height = 385;

		servo_programmer  = new dynamixel_servo_programmer(this, "Servo programmer");
        servo_programmer->setGeometry(QRect(dynamixel_servo_start_x-305, dynamixel_servo_start_y, 300, 255));

		double servo_2_ethanol_preset_angle_1 = 225;
		double servo_2_ethanol_preset_angle_2 = 500;
		double servo_2_ethanol_pos_init = servo_2_ethanol_preset_angle_1;
        servo_2_ethanol = new dynamixel_servo(this, "Ethanol", 2, 0, 1023, 1, servo_2_ethanol_pos_init, 1, 512, 1, 512, servo_2_ethanol_preset_angle_1, servo_2_ethanol_preset_angle_2, servo_programmer);
        servo_2_ethanol->setGeometry(QRect(dynamixel_servo_start_x + 0*(dynamixel_servo_width + dynamixel_servo_width_inc), dynamixel_servo_start_y, dynamixel_servo_width, dynamixel_servo_height));

		double servo_3_air_preset_angle_1 = 364;
		double servo_3_air_preset_angle_2 = 300;
		double servo_3_air_pos_init = servo_3_air_preset_angle_1;
        servo_3_air = new dynamixel_servo(this, "Air", 3, 0, 1023, 1, servo_3_air_pos_init, 1, 1023, 1, 1023, servo_3_air_preset_angle_1, servo_3_air_preset_angle_2, servo_programmer);
        servo_3_air->setGeometry(QRect(dynamixel_servo_start_x + 1*(dynamixel_servo_width + dynamixel_servo_width_inc), dynamixel_servo_start_y, dynamixel_servo_width, dynamixel_servo_height));

		double servo_4_restraint_preset_angle_1 = 605; //CLOSED
		double servo_4_restraint_preset_angle_2 = 205; //OPEN
		double servo_4_restraint_pos_init = servo_4_restraint_preset_angle_1;
        servo_4_restraint = new dynamixel_servo(this, "Restraint", 4, 0, 1023, 1, servo_4_restraint_pos_init, 1, 512, 1, 256, servo_4_restraint_preset_angle_1, servo_4_restraint_preset_angle_2, servo_programmer);
        servo_4_restraint->setGeometry(QRect(dynamixel_servo_start_x + 2*(dynamixel_servo_width + dynamixel_servo_width_inc), dynamixel_servo_start_y, dynamixel_servo_width, dynamixel_servo_height));
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////  Debug plot where we plot whatever variable we need to visualize for debugging
		mainwindow_debug_var_0 = 0;
		mainwindow_debug_var_1 = 0;
		debug_plot_mutex = new QMutex;
		debug_plot_min_val = -100.0;
		debug_plot_max_val = 100.0;
		debug_plot_data_vec.push_back(&mainwindow_debug_var_0);
		//debug_plot_data_vec.push_back(&mainwindow_debug_var_1);
		debug_plot = new DataPlot(debug_plot_data_vec, 1, debug_plot_min_val, debug_plot_max_val, 175, QString("Debug"), "ticks", 0, this, debug_plot_mutex); //NULL used to be stepper_mutex
		debug_plot->setGeometry(QRect(5, 1440-185, 370, 175));
		///////////////////////////////////////////////////////
		
		///////////////////////////////////////////////////////
		velocity_bessel_filter = new bessel_filter(this, 8); 
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////
		force_pop_detector = new my_force_pop_detector(this); 
		force_pop_detector->setGeometry(QRect(400,1175, 1300, 260));
		///////////////////////////////////////////////////////
		
		///////////////////////////////////////////////////////
        int stereo_camera_electronics_controller_QT_width = 500;
        int stereo_camera_electronics_controller_QT_height = 325;
        stereo_camera_electronics_controller_QT = new my_stereo_camera_electronics_controller_QT(this, "Stereo Camera Electronics Controller QT", stereo_camera_electronics_controller_QT_width, stereo_camera_electronics_controller_QT_height);
        stereo_camera_electronics_controller_QT->setGeometry(QRect(1770, 1440-stereo_camera_electronics_controller_QT_height-5, stereo_camera_electronics_controller_QT_width, stereo_camera_electronics_controller_QT_height));
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////
		int LED_board_programmer_width = 370;
        int LED_board_programmer_height = 415;
        LED_board_programmer = new my_LED_board_programmer(this, "LED Board programmer", LED_board_programmer_width, LED_board_programmer_height);
        LED_board_programmer->setGeometry(QRect(1900,690,LED_board_programmer_width, LED_board_programmer_height));
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////
        foot_pedal_USB_stinky = new my_foot_pedal_USB_stinky(this, "USB foot pedal");
        foot_pedal_USB_stinky->setGeometry(QRect(1290, 690, 500, 200));
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////
		this->setFocusPolicy(Qt::StrongFocus);
		///////////////////////////////////////////////////////

		///////////////////////////////////////////////////////
        start_or_end_program_flag = 1;
        gui_timer_id = startTimer(100); //33 (30Hz) creates laggy graphics, 100 has much better results.
		///////////////////////////////////////////////////////
}


MainWindow::~MainWindow()
{

}

void MainWindow::keyPressEvent(QKeyEvent *incoming_key_event) //Function must be named "keyPressEvent" to properly overide the normal even handler and operate correctly.
{
	
	incoming_key_event->accept();
	//cout << "Key event in MainWindow: " << incoming_key_event->text().toStdString() << endl;
	incoming_key_event->ignore();

	QWidget::keyPressEvent(incoming_key_event); //This lien is recommended by the QT class documentation.
}

void MainWindow::timerEvent(QTimerEvent *event)
{
        if(start_or_end_program_flag != -1)
        {

			if(data_log->save_data_state == 1)
			{
				end_program_button->setEnabled(0);
			}
			else
			{
				end_program_button->setEnabled(1);
			}

			servo_loop_frequency = servo_loop_frequency_slider->getSliderVal();

			data_log->timer_update(event);

			mainwindow_current_global_time_label->setText("Current DAQ Global Time: " + QString::number(my_DAQ->current_global_time));
			mainwindow_debug_label->setText("mainwindow Debug: " + QString::number(mainwindow_debug_var_0));
			debug_plot->update_data_plot(event);

            needle_driver_programmer->timer_update(event);

			ATI_assistoBot->timer_update(event);
			ATI_needleBot->timer_update(event);

			velocity_bessel_filter->timer_update(event);

			my_DAQ->timer_update(event);

			//stepper_Rx_mutex->lock();
            stepper_0->timer_update(event);
            stepper_1->timer_update(event);
            stepper_2->timer_update(event);
            stepper_programmer->timer_update(event);
			my_BP_cuff->timer_update(event);

            //stepper_Rx_mutex->unlock();

			servo_2_ethanol->timer_update(event);
            servo_3_air->timer_update(event);
            servo_4_restraint->timer_update(event);
            servo_programmer->timer_update(event);

			for(int motor_num = 0; motor_num < motor_GUI_vec.size(); motor_num++)
			{
				motor_GUI_vec[motor_num]->timer_update(event);
			}
			
			needleBot_controller_instance->timer_update(event);

			stereo_camera_electronics_controller_QT->timer_update(event);

			LED_board_programmer->timer_update(event);

			force_pop_detector->timer_update(event);

			foot_pedal_USB_stinky->timer_update(event);

			if(data_log->focus_needs_to_be_restored_to_other_widgets_flag == 1)
			{
				foot_pedal_USB_stinky->my_setFocus_function(1);
				data_log->focus_needs_to_be_restored_to_other_widgets_flag = 0;
			}

			if(needleBot_controller_instance->what_needs_to_happen_to_foot_pedal_focus == 1)
			{
				foot_pedal_USB_stinky->switch_3_BOOL_val = 1;
				foot_pedal_USB_stinky->my_setFocus_function(1);
				needleBot_controller_instance->what_needs_to_happen_to_foot_pedal_focus = 0;
			}

			/////////////////////////////////////////////
			M_HAP_ARM->timer_update(event);
			
            if(stepper_programmer->isJointCoords() == 1)
            {
				stepper_0->slider_box_pos->setEnabled(1);
				stepper_1->slider_box_pos->setEnabled(1);
				stepper_2->slider_box_pos->setEnabled(1);

				stepper_programmer->slider_box_manual_x->setEnabled(0);
				stepper_programmer->slider_box_manual_y->setEnabled(0);
				stepper_programmer->slider_box_manual_z->setEnabled(0);
            }
            else
            {
				stepper_0->slider_box_pos->setEnabled(0);
				stepper_1->slider_box_pos->setEnabled(0);
				stepper_2->slider_box_pos->setEnabled(0);

				stepper_programmer->slider_box_manual_x->setEnabled(1);
				stepper_programmer->slider_box_manual_y->setEnabled(1);
				stepper_programmer->slider_box_manual_z->setEnabled(1);
            }

			stepper_0->slider_box_pos->setVal(M_HAP_ARM->joint_vector_rad[0]);
			stepper_1->slider_box_pos->setVal(M_HAP_ARM->joint_vector_rad[1]);
			stepper_2->slider_box_pos->setVal(M_HAP_ARM->joint_vector_rad[2]);

			if(stepper_programmer->manual_control_flag == 0 || needleBot_controller_instance->auto_control_flag == 1)
			{
				stepper_programmer->slider_box_manual_x->setVal(M_HAP_ARM->EE.x);
				stepper_programmer->slider_box_manual_y->setVal(M_HAP_ARM->EE.y);
				stepper_programmer->slider_box_manual_z->setVal(M_HAP_ARM->EE.z);
			}
			/////////////////////////////////////////////




        }
        else if(start_or_end_program_flag == -1)
        {
                killTimer(gui_timer_id);

				my_DAQ->close();

				//ATI_assistoBot->close();
				
				//M_HAP_ARM->close();
				
				///////////////////////////////// all objects which contain an RLserial COM connection must close for them to reopen properly on the next run of the program.
				servo_programmer->close();
				stepper_programmer->close();
				needle_driver_programmer->close();
				stereo_camera_electronics_controller_QT->close();
				LED_board_programmer->close();
				/////////////////////////////////

				//force_pop_detector->close();

                qApp->closeAllWindows(); //must be called from gui thread
        }
}


void MainWindow::end_program()
{
	if(data_log->save_data_state == 0)
	{
		start_or_end_program_flag = -1;
	}
	else
	{
		cout << "Saving data, can't exit program yet." << endl;
	}
}

void MainWindow::start_program()
{
        start_or_end_program_flag = 1;
}
