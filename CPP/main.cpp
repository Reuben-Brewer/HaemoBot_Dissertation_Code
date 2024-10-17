#include <QtGui/QApplication>
#include "mainwindow.h"
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <cmath>
#include <QMutex>
#include <iostream>

#define pi_define 3.141592654

using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
MainWindow *mainWindow;
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyThread_robot_thread : public QThread
{
    public:
        virtual void run();
};

void MyThread_robot_thread::run()
{
    printf("Entering robot thread. \n");

	double current_servo_loop_global_time = 0;
	double last_servo_loop_global_time = 0;
	std::vector<bool> first_time_entering_PID_loop_after_being_calibrated_flag_vec = std::vector<bool>(6, 1);

	mainWindow->motor_GUI_vec[2]->calibration_performed_flag = 1; //Set the Z to calibrated at the front end.
	mainWindow->motor_GUI_vec[5]->calibration_performed_flag = 1; //Set the roll to calibrated at the front end.
	mainWindow->motor_GUI_vec[6]->calibration_performed_flag = 1; //Set the cath ins to calibrated at the front end.

    while(mainWindow->start_or_end_program_flag == 1)
    {
		current_servo_loop_global_time = mainWindow->my_DAQ->update_global_clock(); //Keep updating the clock until we see enough time elapse.

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Start of Servo Loop Functions
		if(current_servo_loop_global_time - last_servo_loop_global_time >= 1/mainWindow->servo_loop_frequency)
		{
			///////////////////////////////////////////////Update DAQ
			mainWindow->my_DAQ->toggle_do_DMT();

			mainWindow->my_DAQ->servo_loop_frequency_set_by_slider = mainWindow->servo_loop_frequency;
			
			mainWindow->my_DAQ->update_encoders();
			mainWindow->my_DAQ->read_di();	

			mainWindow->needleBot_controller_instance->updateOmniInput(mainWindow->M_HAP_ARM->PosNeedleBot);
			mainWindow->needleBot_controller_instance->updateActualPosVelAccel(mainWindow->my_DAQ->encoder_vec, mainWindow->my_DAQ->velocity_bessel_smoothed_vec, std::vector<double>(7,0), mainWindow->my_DAQ->current_global_time); //Leaving the acceleration vector blank for now.
			mainWindow->needleBot_controller_instance->calculate_robot_line();
			mainWindow->needleBot_controller_instance->updateDesiredPosVelAccel();

			///////////////////////////////////////////////Update motors
			for(int motor_number_index = 0; motor_number_index < mainWindow->motor_GUI_vec.size(); motor_number_index++)
			{
				mainWindow->motor_GUI_vec[motor_number_index]->motor_enable_hardware_flag = mainWindow->my_DAQ->motor_enable_hardware_vec[motor_number_index];
				mainWindow->motor_GUI_vec[motor_number_index]->update_position_and_velocity(mainWindow->my_DAQ->encoder_vec[motor_number_index], mainWindow->my_DAQ->velocity_bessel_smoothed_vec[motor_number_index]);
				mainWindow->motor_GUI_vec[motor_number_index]->update_home_and_limit_flags(mainWindow->my_DAQ->home_flag_vec[motor_number_index], mainWindow->my_DAQ->neg_slot_flag_vec[motor_number_index], mainWindow->my_DAQ->neg_mag_flag_vec[motor_number_index], mainWindow->my_DAQ->pos_slot_flag_vec[motor_number_index], mainWindow->my_DAQ->pos_mag_flag_vec[motor_number_index]);
				
				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  JOINT HASN'T YET STARTED CALIBRATION
				if(mainWindow->motor_GUI_vec[motor_number_index]->calibration_performed_flag == -1) 
				{
					mainWindow->motor_GUI_vec[motor_number_index]->home_flag_calibration_starting_value = mainWindow->motor_GUI_vec[motor_number_index]->home_flag;
					if(mainWindow->motor_GUI_vec[motor_number_index]->home_flag_calibration_starting_value == 0)
					{
						mainWindow->motor_GUI_vec[motor_number_index]->velocity_desired = mainWindow->motor_GUI_vec[motor_number_index]->calibration_speed_for_starting_ZERO_on_home_sensor;
					}
					else if(mainWindow->motor_GUI_vec[motor_number_index]->home_flag_calibration_starting_value == 1)
					{
						mainWindow->motor_GUI_vec[motor_number_index]->velocity_desired = mainWindow->motor_GUI_vec[motor_number_index]->calibration_speed_for_starting_ONE_on_home_sensor;
					}
					else if(mainWindow->motor_GUI_vec[motor_number_index]->home_flag_calibration_starting_value == -1) //For joints that don't have homing flags
					{
						mainWindow->motor_GUI_vec[motor_number_index]->velocity_desired = mainWindow->motor_GUI_vec[motor_number_index]->calibration_speed_for_starting_ZERO_on_home_sensor;
					}
		
					mainWindow->motor_GUI_vec[motor_number_index]->setControlMode(0);
					mainWindow->motor_GUI_vec[motor_number_index]->calibration_performed_flag = 0;
				}
				////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  JOINT IS UNDERGOING CALIBRATION
				else if(mainWindow->motor_GUI_vec[motor_number_index]->calibration_performed_flag == 0) 
				{
					if(mainWindow->motor_GUI_vec[motor_number_index]->home_flag == mainWindow->motor_GUI_vec[motor_number_index]->home_flag_calibration_starting_value)
					{
						mainWindow->motor_GUI_vec[motor_number_index]->compute_errors_and_voltage(); //Pure velocity controller
					}
					else
					{
						mainWindow->my_DAQ->encoder_offset_at_calibration_vec[motor_number_index] = mainWindow->my_DAQ->encoder_vec[motor_number_index];
						mainWindow->motor_GUI_vec[motor_number_index]->velocity_desired = 0; 
						mainWindow->motor_GUI_vec[motor_number_index]->calibration_performed_flag = 1;
						mainWindow->motor_GUI_vec[motor_number_index]->setControlMode(1);
						cout << "Homed axis " << motor_number_index << endl;
					}
					
				}
				////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  JOINT IS ALREADY CALIBRATED AND IS UNDER PID CONTROL
				else if(mainWindow->motor_GUI_vec[motor_number_index]->calibration_performed_flag == 1) 
				{
					if(first_time_entering_PID_loop_after_being_calibrated_flag_vec[motor_number_index] == 1)
					{
						mainWindow->my_DAQ->encoder_offset_at_calibration_vec[motor_number_index] += mainWindow->motor_GUI_vec[motor_number_index]->position_offset_for_needleBot_to_assistoBot_calibration;
						first_time_entering_PID_loop_after_being_calibrated_flag_vec[motor_number_index] = 0;
					}

					mainWindow->motor_GUI_vec[motor_number_index]->position_desired = mainWindow->needleBot_controller_instance->PosDesired[motor_number_index];
					mainWindow->motor_GUI_vec[motor_number_index]->velocity_desired = mainWindow->needleBot_controller_instance->VelDesired[motor_number_index]; ////////////ADD THIS BACK IN
					mainWindow->motor_GUI_vec[motor_number_index]->acceleration_desired = mainWindow->needleBot_controller_instance->AccelDesired[motor_number_index];
					
					mainWindow->motor_GUI_vec[motor_number_index]->compute_errors_and_voltage();
				}
				////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

				mainWindow->my_DAQ->motor_enable_software_vec[motor_number_index] = mainWindow->motor_GUI_vec[motor_number_index]->motor_enable_software_flag;
				mainWindow->my_DAQ->analog_voltage_to_write_AO_vec[motor_number_index] = mainWindow->motor_GUI_vec[motor_number_index]->voltage_to_write_limited;
			}

			if(mainWindow->needleBot_controller_instance->shouldOutsideThreadsYieldControlToNeedleBotController() == 1)
			{
				mainWindow->needle_driver_programmer->setCathLoopState(mainWindow->needleBot_controller_instance->CathLoopState);
				mainWindow->needle_driver_programmer->setCathSwingArmState(mainWindow->needleBot_controller_instance->CathSwingArmState);
			}

			//////////////////////////////////////////////////////////////////////// Foot pedal controls of ND and NeedleBot
			if(mainWindow->needleBot_controller_instance->haptic_control_flag == 1)
			{
				mainWindow->needle_driver_programmer->setCathLoopState(mainWindow->foot_pedal_USB_stinky->switch_4_BOOL_val);
				mainWindow->needle_driver_programmer->setCathSwingArmState(mainWindow->foot_pedal_USB_stinky->switch_2_BOOL_val);
			}

			mainWindow->foot_pedal_USB_stinky->keyEvent_mutex->lock();
					if((mainWindow->needleBot_controller_instance->manual_control_flag == 0 && mainWindow->needleBot_controller_instance->haptic_control_flag == 1) || (mainWindow->needleBot_controller_instance->manual_control_flag == 1 && mainWindow->needleBot_controller_instance->haptic_control_flag == 0))
					{
						if(mainWindow->foot_pedal_USB_stinky->getKeyboardFocusValue() == 1)
						{
							
								mainWindow->needleBot_controller_instance->set_haptic_control_flag(mainWindow->foot_pedal_USB_stinky->switch_3_BOOL_val);
								mainWindow->needleBot_controller_instance->set_manual_control_flag(!mainWindow->foot_pedal_USB_stinky->switch_3_BOOL_val);
							
						}
					}
			mainWindow->foot_pedal_USB_stinky->keyEvent_mutex->unlock();
			////////////////////////////////////////////////////////////////////////

			mainWindow->my_DAQ->write_ao();
			///////////////////////////////////////////////
			
			///////////////////////////////////////////////Update ATI sensors and objects
			double time_to_wait_after_startup_to_bias_the_force_sensors = 5.0;
			if(current_servo_loop_global_time >= time_to_wait_after_startup_to_bias_the_force_sensors && mainWindow->ATI_assistoBot->force_sensor_has_been_biased_at_startup_flag == 0)
			{
				mainWindow->ATI_assistoBot->BiasVoltage();
				mainWindow->ATI_assistoBot->force_sensor_has_been_biased_at_startup_flag = 1;
			}
			if(current_servo_loop_global_time >= time_to_wait_after_startup_to_bias_the_force_sensors && mainWindow->ATI_needleBot->force_sensor_has_been_biased_at_startup_flag == 0)
			{
				mainWindow->ATI_needleBot->BiasVoltage();
				mainWindow->ATI_needleBot->force_sensor_has_been_biased_at_startup_flag = 1;
			}

			mainWindow->my_DAQ->update_ATI();
			mainWindow->ATI_assistoBot->update(mainWindow->my_DAQ->ATI_assistoBot_raw_vec);
			mainWindow->ATI_needleBot->update(mainWindow->my_DAQ->ATI_needleBot_raw_vec);

			mainWindow->needleBot_controller_instance->updateAssistoBotForce(mainWindow->ATI_assistoBot->ATI_FT_smoothed);

			mainWindow->force_pop_detector->update(mainWindow->ATI_needleBot->ATI_FT_smoothed, mainWindow->servo_loop_frequency); //WE RESET THE POP LATCH IN THE CODE A LITTLE BIT BELOW HERE WHERE WE AUTO CREATE AND SAVE THE DATA LOGGER FILE IN AUTO MODE
			mainWindow->needleBot_controller_instance->updateInsertionForcePopState(mainWindow->force_pop_detector->pop_detection_state_latched);

			mainWindow->needle_driver_programmer->stringpot_voltage = mainWindow->my_DAQ->needle_driver_stringpot_voltage_raw;
			mainWindow->needle_driver_programmer->stringpot_position = mainWindow->my_DAQ->needle_driver_stringpot_position;
			///////////////////////////////////////////////

			///////////////////////////////////////////////
			///////////////////////////////////////////////Log Data

			////////////////////////// Automatically save data file for the duration of each automatic line insertion
				if(mainWindow->needleBot_controller_instance->what_needs_to_happen_to_data_logger_file == 1)
				{
					mainWindow->data_log->actual_file_name_to_save_with = QString("auto_insertion_" + mainWindow->data_log->file_name_input);
					mainWindow->data_log->change_record_state(1);

					mainWindow->force_pop_detector->reset_pop_detection_latch(); ///////////////////////////////////////////////////////DOING THIS HERE, TOO SO AS NOT TO HAVE TO REPLCIATE CODE
					mainWindow->force_pop_detector->take_force_snapshot(); ///////////////////////////////////////////////////////DOING THIS HERE, TOO SO AS NOT TO HAVE TO REPLCIATE CODE
					mainWindow->needleBot_controller_instance->what_needs_to_happen_to_data_logger_file = 0;
				}
				else if(mainWindow->needleBot_controller_instance->what_needs_to_happen_to_data_logger_file == -1)
				{
					mainWindow->data_log->change_record_state(0);
					mainWindow->needleBot_controller_instance->what_needs_to_happen_to_data_logger_file = 0;
				}
				else if(mainWindow->needleBot_controller_instance->what_needs_to_happen_to_data_logger_file == 0)
				{
					//Do nothing special.
				}
			//////////////////////////

			double temp_current_global_time = mainWindow->my_DAQ->current_global_time;
			if(mainWindow->data_log->save_data_state == 1)
			{
				if(mainWindow->data_log->check_if_time_to_add_another_data_point(temp_current_global_time))
				{
					char data_string[1200];
			
					///////////////// IF YOU GET A VECTOR ERROR FROM THE DATA LOGGING, THEN YOU NEED TO INCREASE THE SIZE OF DATA_STRING
					//sprintf(data_string, " 1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33 ,34 ,35 ,36 ,37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62\n", temp_current_global_time, mainWindow->my_DAQ->encoder_vec[0], mainWindow->my_DAQ->encoder_vec[1], mainWindow->my_DAQ->encoder_vec[2], mainWindow->my_DAQ->encoder_vec[3], mainWindow->my_DAQ->encoder_vec[4], mainWindow->my_DAQ->encoder_vec[5], mainWindow->my_DAQ->encoder_vec[6], mainWindow->motor_GUI_vec[0]->velocity_actual, mainWindow->motor_GUI_vec[1]->velocity_actual, mainWindow->motor_GUI_vec[2]->velocity_actual, mainWindow->motor_GUI_vec[3]->velocity_actual, mainWindow->motor_GUI_vec[4]->velocity_actual, mainWindow->motor_GUI_vec[5]->velocity_actual, mainWindow->motor_GUI_vec[6]->velocity_actual, mainWindow->needleBot_controller_instance->PosDesired[0], mainWindow->needleBot_controller_instance->PosDesired[1], mainWindow->needleBot_controller_instance->PosDesired[2], mainWindow->needleBot_controller_instance->PosDesired[3], mainWindow->needleBot_controller_instance->PosDesired[4], mainWindow->needleBot_controller_instance->PosDesired[5], mainWindow->needleBot_controller_instance->PosDesired[6], mainWindow->needleBot_controller_instance->auto_line_assistoBot_posDesired_vec[0], mainWindow->needleBot_controller_instance->auto_line_assistoBot_posDesired_vec[1], mainWindow->needleBot_controller_instance->auto_line_assistoBot_posDesired_vec[2], mainWindow->ATI_assistoBot->ATI_FT_smoothed[0], mainWindow->ATI_assistoBot->ATI_FT_smoothed[1], mainWindow->ATI_assistoBot->ATI_FT_smoothed[2], mainWindow->ATI_assistoBot->ATI_FT_smoothed[3], mainWindow->ATI_assistoBot->ATI_FT_smoothed[4], mainWindow->ATI_assistoBot->ATI_FT_smoothed[5], mainWindow->ATI_needleBot->ATI_FT_smoothed[0], mainWindow->ATI_needleBot->ATI_FT_smoothed[1], mainWindow->ATI_needleBot->ATI_FT_smoothed[2], mainWindow->ATI_needleBot->ATI_FT_smoothed[3], mainWindow->ATI_needleBot->ATI_FT_smoothed[4], mainWindow->ATI_needleBot->ATI_FT_smoothed[5], mainWindow->force_pop_detector->force_pop_detection_threshold, mainWindow->force_pop_detector->force_derivative_pop_detection_threshold, mainWindow->force_pop_detector->pop_detection_state, mainWindow->force_pop_detector->pop_detection_state_latched, mainWindow->force_pop_detector->ATI_FT_derivative_smoothed[0], mainWindow->force_pop_detector->ATI_FT_derivative_smoothed[1], mainWindow->force_pop_detector->ATI_FT_derivative_smoothed[2], mainWindow->force_pop_detector->ATI_FT_derivative_smoothed[3], mainWindow->force_pop_detector->ATI_FT_derivative_smoothed[4], mainWindow->force_pop_detector->ATI_FT_derivative_smoothed[5], mainWindow->force_pop_detector->ATI_FT_in_OFFSET_BY_SNAPSHOT[0], mainWindow->force_pop_detector->ATI_FT_in_OFFSET_BY_SNAPSHOT[1], mainWindow->force_pop_detector->ATI_FT_in_OFFSET_BY_SNAPSHOT[2], mainWindow->force_pop_detector->ATI_FT_in_OFFSET_BY_SNAPSHOT[3], mainWindow->force_pop_detector->ATI_FT_in_OFFSET_BY_SNAPSHOT[4], mainWindow->force_pop_detector->ATI_FT_in_OFFSET_BY_SNAPSHOT[5], mainWindow->needleBot_controller_instance->insertion_depth_before_detecting_pop, mainWindow->needleBot_controller_instance->current_insertion_depth, mainWindow->needle_driver_programmer->accelData[0], mainWindow->needle_driver_programmer->accelData[1], mainWindow->needle_driver_programmer->accelData[2], mainWindow->M_HAP_ARM->EE[0], mainWindow->M_HAP_ARM->EE[1], mainWindow->M_HAP_ARM->EE[2], mainWindow->needleBot_controller_instance->auto_insertion_substate);
					  sprintf(data_string, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", temp_current_global_time, mainWindow->my_DAQ->encoder_vec[0], mainWindow->my_DAQ->encoder_vec[1], mainWindow->my_DAQ->encoder_vec[2], mainWindow->my_DAQ->encoder_vec[3], mainWindow->my_DAQ->encoder_vec[4], mainWindow->my_DAQ->encoder_vec[5], mainWindow->my_DAQ->encoder_vec[6], mainWindow->motor_GUI_vec[0]->velocity_actual, mainWindow->motor_GUI_vec[1]->velocity_actual, mainWindow->motor_GUI_vec[2]->velocity_actual, mainWindow->motor_GUI_vec[3]->velocity_actual, mainWindow->motor_GUI_vec[4]->velocity_actual, mainWindow->motor_GUI_vec[5]->velocity_actual, mainWindow->motor_GUI_vec[6]->velocity_actual, mainWindow->needleBot_controller_instance->PosDesired[0], mainWindow->needleBot_controller_instance->PosDesired[1], mainWindow->needleBot_controller_instance->PosDesired[2], mainWindow->needleBot_controller_instance->PosDesired[3], mainWindow->needleBot_controller_instance->PosDesired[4], mainWindow->needleBot_controller_instance->PosDesired[5], mainWindow->needleBot_controller_instance->PosDesired[6], mainWindow->needleBot_controller_instance->auto_line_assistoBot_posDesired_vec[0], mainWindow->needleBot_controller_instance->auto_line_assistoBot_posDesired_vec[1], mainWindow->needleBot_controller_instance->auto_line_assistoBot_posDesired_vec[2], mainWindow->ATI_assistoBot->ATI_FT_smoothed[0], mainWindow->ATI_assistoBot->ATI_FT_smoothed[1], mainWindow->ATI_assistoBot->ATI_FT_smoothed[2], mainWindow->ATI_assistoBot->ATI_FT_smoothed[3], mainWindow->ATI_assistoBot->ATI_FT_smoothed[4], mainWindow->ATI_assistoBot->ATI_FT_smoothed[5], mainWindow->ATI_needleBot->ATI_FT_smoothed[0], mainWindow->ATI_needleBot->ATI_FT_smoothed[1], mainWindow->ATI_needleBot->ATI_FT_smoothed[2], mainWindow->ATI_needleBot->ATI_FT_smoothed[3], mainWindow->ATI_needleBot->ATI_FT_smoothed[4], mainWindow->ATI_needleBot->ATI_FT_smoothed[5], mainWindow->force_pop_detector->force_pop_detection_threshold, mainWindow->force_pop_detector->force_derivative_pop_detection_threshold, mainWindow->force_pop_detector->pop_detection_state, mainWindow->force_pop_detector->pop_detection_state_latched, mainWindow->force_pop_detector->ATI_FT_derivative_smoothed[0], mainWindow->force_pop_detector->ATI_FT_derivative_smoothed[1], mainWindow->force_pop_detector->ATI_FT_derivative_smoothed[2], mainWindow->force_pop_detector->ATI_FT_derivative_smoothed[3], mainWindow->force_pop_detector->ATI_FT_derivative_smoothed[4], mainWindow->force_pop_detector->ATI_FT_derivative_smoothed[5], mainWindow->force_pop_detector->ATI_FT_in_OFFSET_BY_SNAPSHOT[0], mainWindow->force_pop_detector->ATI_FT_in_OFFSET_BY_SNAPSHOT[1], mainWindow->force_pop_detector->ATI_FT_in_OFFSET_BY_SNAPSHOT[2], mainWindow->force_pop_detector->ATI_FT_in_OFFSET_BY_SNAPSHOT[3], mainWindow->force_pop_detector->ATI_FT_in_OFFSET_BY_SNAPSHOT[4], mainWindow->force_pop_detector->ATI_FT_in_OFFSET_BY_SNAPSHOT[5], mainWindow->needleBot_controller_instance->insertion_depth_before_detecting_pop, mainWindow->needleBot_controller_instance->current_insertion_depth, mainWindow->needle_driver_programmer->accelData[0], mainWindow->needle_driver_programmer->accelData[1], mainWindow->needle_driver_programmer->accelData[2], mainWindow->M_HAP_ARM->EE[0], mainWindow->M_HAP_ARM->EE[1], mainWindow->M_HAP_ARM->EE[2], mainWindow->needleBot_controller_instance->auto_insertion_substate); 
			
					mainWindow->data_log->data_to_write.push_back(data_string);
					mainWindow->data_log->time_last_data_point_added = temp_current_global_time;
					//cout << "Pushed new data" << endl;
				}
			}
			///////////////////////////////////////////////
			///////////////////////////////////////////////

			mainWindow->mainwindow_debug_var_0 = mainWindow->needleBot_controller_instance->auto_insertion_substate;
			//mainWindow->mainwindow_debug_var_1 = mainWindow->my_DAQ->encoder_INCREMENTAL_vec[6];
			

			last_servo_loop_global_time = current_servo_loop_global_time;
		}
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////End of Servo Loop Functions
	}

	mainWindow->my_DAQ->zero_ao();

    printf("Exiting robot thread. \n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyThread_needle_driver_Rx : public QThread
{
    public:
        virtual void run();
};

void MyThread_needle_driver_Rx::run()
{
    bool message_done_flag = false;

    unsigned long Rx_sleep_time_ms = 1;//milliseconds
    unsigned long Rx_sleep_time_us;//microseconds
    Rx_sleep_time_us = 1000*Rx_sleep_time_ms;


    while(mainWindow->start_or_end_program_flag == 1)
    {
        mainWindow->needle_driver_programmer->readMessage(message_done_flag);
        //Sleep(Rx_sleep_time_ms); //windows only, it's usleep in linux //no longer needed at baud rate = 0.5MBS 11/20/2012


        //printf("in");
        if(message_done_flag == true)
        {
            message_done_flag = false;
            mainWindow->needle_driver_programmer->NeedleDriverRxMutex->lock();

                //printf("debug from micro: %lf %lf\n", mainWindow->needle_driver_programmer->touchState, mainWindow->needle_driver_programmer->time_from_micro);
            mainWindow->needle_driver_programmer->NeedleDriverRxMutex->unlock();
        }
    }

    printf("Exiting needle driver Rx thread. \n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyThread_needle_driver_Tx : public QThread
{
    public:
        virtual void run();
};

void MyThread_needle_driver_Tx::run()
{
    unsigned long Tx_sleep_time = 1;//milliseconds

    while(mainWindow->start_or_end_program_flag == 1)
    {
        mainWindow->needle_driver_programmer->commandStates();
        Sleep(Tx_sleep_time); //windows only, it's usleep in linux

    }

    printf("Exiting needle driver Tx thread. \n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyThread_stepper_Tx : public QThread
{
    public:
        virtual void run();
};

void MyThread_stepper_Tx::run()
{
    double goal_pressure;
    std::vector<double> goal_pos, goal_vel;
    std::vector<bool> goal_mode;
    std::vector<bool> motors_to_be_homed;
    std::vector<unsigned char> motors_home_speed;
    motors_to_be_homed.push_back(0);
    motors_to_be_homed.push_back(0);
    motors_to_be_homed.push_back(0);
    motors_home_speed.push_back(10);
    motors_home_speed.push_back(10);
    motors_home_speed.push_back(10);

    unsigned long Tx_sleep_time = 1; //mS

    while(mainWindow->start_or_end_program_flag == 1)
    {


        if(mainWindow->stepper_programmer->message_to_be_sent == 1)
        {

            if(mainWindow->stepper_programmer->isManualControl() == false)
            {
            //    mainWindow->stepper_0->goal_stepper_pos_double = mainWindow->M_HAP_ARM->joint_vector_num_full_steps[0];
            //    mainWindow->stepper_1->goal_stepper_pos_double = mainWindow->M_HAP_ARM->joint_vector_num_full_steps[1];
            //    mainWindow->stepper_2->goal_stepper_pos_double = mainWindow->M_HAP_ARM->joint_vector_num_full_steps[2];
            }
            else
            {
             //  mainWindow->stepper_0->goal_stepper_pos_double = mainWindow->stepper_0->slider_box_pos->value;
             //  mainWindow->stepper_1->goal_stepper_pos_double = mainWindow->stepper_1->slider_box_pos->value;
             //  mainWindow->stepper_2->goal_stepper_pos_double = mainWindow->stepper_2->slider_box_pos->value;
            }

			//commented out on 7/27/14 @ 6:41pm while debugging use not receiving Rx messages or plotting correctly
            goal_pos.push_back(mainWindow->stepper_0->goal_full_steps);
            goal_pos.push_back(mainWindow->stepper_1->goal_full_steps);
            goal_pos.push_back(mainWindow->stepper_2->goal_full_steps);
			

			/////////////////////////////// added 7/27/14 @ 6:41pm while debugging use not receiving Rx messages or plotting correctly
			//goal_pos.push_back(mainWindow->stepper_0->goal_stepper_pos_double);
            //goal_pos.push_back(mainWindow->stepper_1->goal_stepper_pos_double);
            //goal_pos.push_back(mainWindow->stepper_2->goal_stepper_pos_double);
			//////////////////////////////

            goal_vel.push_back(mainWindow->stepper_0->goal_stepper_vel_double);
            goal_vel.push_back(mainWindow->stepper_1->goal_stepper_vel_double);
            goal_vel.push_back(mainWindow->stepper_2->goal_stepper_vel_double);

            goal_mode.push_back(mainWindow->stepper_0->step_mode);
            goal_mode.push_back(mainWindow->stepper_1->step_mode);
            goal_mode.push_back(mainWindow->stepper_2->step_mode);

            goal_pressure = mainWindow->my_BP_cuff->goal_pressure_double;

            mainWindow->stepper_programmer->move_stepper(goal_pos, goal_vel, goal_mode, goal_pressure);

            Sleep(Tx_sleep_time);//mS

            goal_pos.clear();
            goal_vel.clear();
            goal_mode.clear();
        }
        else if(mainWindow->stepper_programmer->message_to_be_sent == 2)
        {

            motors_to_be_homed[0] = mainWindow->stepper_0->toBeHomed;
            motors_to_be_homed[1] = mainWindow->stepper_1->toBeHomed;
            motors_to_be_homed[2] = mainWindow->stepper_2->toBeHomed;

            unsigned int temp0 = mainWindow->stepper_0->home_speed;
            unsigned int temp1 = mainWindow->stepper_1->home_speed;
            unsigned int temp2 = mainWindow->stepper_2->home_speed;
            printf("%d %d %d \n", temp0, temp1, temp2);

            motors_home_speed[0] = mainWindow->stepper_0->home_speed;
            motors_home_speed[1] = mainWindow->stepper_1->home_speed;
            motors_home_speed[2] = mainWindow->stepper_2->home_speed;

            mainWindow->stepper_programmer->home_steppers(motors_to_be_homed, motors_home_speed);
            Sleep(Tx_sleep_time);//mS

            mainWindow->stepper_programmer->message_to_be_sent = 1;
        }
    }

    printf("Exiting stepper Tx thread. \n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyThread_stepper_Rx : public QThread
{
    public:
        virtual void run();
};

void MyThread_stepper_Rx::run()
{
    bool message_done_flag = false;
    std::vector<double> actual_pos;
    actual_pos.push_back(0);
    actual_pos.push_back(0);
    actual_pos.push_back(0);
    double BP;
    std::vector<signed char> homed_state;
    homed_state.push_back(-1);
    homed_state.push_back(-1);
    homed_state.push_back(-1);
	double Rx_stepper_programmer_sleep_time_ms = 1;

    while(mainWindow->start_or_end_program_flag == 1)
    {
        mainWindow->stepper_programmer->readMessage(homed_state, actual_pos, BP, message_done_flag);
		//Sleep(Rx_stepper_programmer_sleep_time_ms); //DO NOT USE THIS LINE: CAUSES DROPPED PACKETS AND NOISE. LAST TESTED 07/27/2014 7:00PM

        if(message_done_flag == true)
        {

            message_done_flag = false;
            mainWindow->stepper_Rx_mutex->lock();
            mainWindow->stepper_0->homed_state = homed_state[0];

            if(homed_state[0] == 1) //The flag for a stepper to be homed only gets cleared once the micro sends back the message that the stepper has been homed
            {
                mainWindow->stepper_0->toBeHomed = 0;
            }
            if(homed_state[1] == 1)
            {
                mainWindow->stepper_1->toBeHomed = 0;
            }
            if(homed_state[2] == 1)
            {
                mainWindow->stepper_2->toBeHomed = 0;
            }

            mainWindow->stepper_0->homed_state = homed_state[0];
            mainWindow->stepper_1->homed_state = homed_state[1];
            mainWindow->stepper_2->homed_state = homed_state[2];
            mainWindow->stepper_0->actual_full_steps = actual_pos[0];
            mainWindow->stepper_1->actual_full_steps = actual_pos[1];
            mainWindow->stepper_2->actual_full_steps = actual_pos[2];
            mainWindow->my_BP_cuff->actual_pressure_double = BP;

			


			//cout << "BP: " << BP << endl;

            //printf("debug from micro: %f \n", mainWindow->stepper_programmer->debug_from_micro);
            mainWindow->stepper_Rx_mutex->unlock();
        }
    }

    printf("Exiting stepper Rx thread. \n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyThread_assistoBot : public QThread
{
    public:
        virtual void run();
};

void MyThread_assistoBot::run()
{

    while(mainWindow->start_or_end_program_flag == 1)
    {
		mainWindow->M_HAP_ARM->updateHapticPose();
		mainWindow->stepper_programmer->updateOmniInput(mainWindow->M_HAP_ARM->PosAssistoBot);
		mainWindow->stepper_programmer->updateDesiredPosVelAccel();

		mainWindow->foot_pedal_USB_stinky->keyEvent_mutex->lock();
				//if((mainWindow->needleBot_controller_instance->manual_control_flag == 0 && mainWindow->needleBot_controller_instance->haptic_control_flag == 1) || (mainWindow->needleBot_controller_instance->manual_control_flag == 1 && mainWindow->needleBot_controller_instance->haptic_control_flag == 0))
				//{
					if(mainWindow->foot_pedal_USB_stinky->getKeyboardFocusValue() == 1)
					{
						mainWindow->stepper_programmer->set_manual_control_flag(mainWindow->foot_pedal_USB_stinky->switch_1_BOOL_val);		
					}
					else
					{
						mainWindow->foot_pedal_USB_stinky->switch_1_BOOL_val = mainWindow->stepper_programmer->manual_control_flag;
					}
				//}
		mainWindow->foot_pedal_USB_stinky->keyEvent_mutex->unlock();


		if(mainWindow->needleBot_controller_instance->shouldOutsideThreadsYieldControlToNeedleBotController() == 1)
		{
			mainWindow->M_HAP_ARM->EE[0] = mainWindow->needleBot_controller_instance->auto_line_assistoBot_posDesired_vec[0];
			mainWindow->M_HAP_ARM->EE[1] = mainWindow->needleBot_controller_instance->auto_line_assistoBot_posDesired_vec[1];
			mainWindow->M_HAP_ARM->EE[2] = mainWindow->needleBot_controller_instance->auto_line_assistoBot_posDesired_vec[2];
			mainWindow->M_HAP_ARM->joint_vector_rad = mainWindow->M_HAP_ARM->InvKinematics(mainWindow->M_HAP_ARM->EE);
		}
		else
		{
			if(mainWindow->stepper_programmer->isManualControl() == false)
			{    
				//mainWindow->M_HAP_ARM->EE = mainWindow->M_HAP_ARM->toolPos_assistoBot;
				mainWindow->M_HAP_ARM->EE[0] = mainWindow->stepper_programmer->Pos_omniInput_offset[0];
				mainWindow->M_HAP_ARM->EE[1] = mainWindow->stepper_programmer->Pos_omniInput_offset[1];
				mainWindow->M_HAP_ARM->EE[2] = mainWindow->stepper_programmer->Pos_omniInput_offset[2];
				mainWindow->M_HAP_ARM->joint_vector_rad = mainWindow->M_HAP_ARM->InvKinematics(mainWindow->M_HAP_ARM->EE);
			}
			else
			{
				if(mainWindow->stepper_programmer->isJointCoords() == true)
				{
					mainWindow->M_HAP_ARM->joint_vector_rad[0] = mainWindow->stepper_0->goal_stepper_pos_double;
					mainWindow->M_HAP_ARM->joint_vector_rad[1] = mainWindow->stepper_1->goal_stepper_pos_double;
					mainWindow->M_HAP_ARM->joint_vector_rad[2] = mainWindow->stepper_2->goal_stepper_pos_double;
					mainWindow->M_HAP_ARM->EE = mainWindow->M_HAP_ARM->FwdKinematics(mainWindow->M_HAP_ARM->joint_vector_rad);
				}
				else
				{
					mainWindow->M_HAP_ARM->EE[0] = mainWindow->stepper_programmer->PosDesired[0];//slider_box_manual_x->getSliderVal();
					mainWindow->M_HAP_ARM->EE[1] = mainWindow->stepper_programmer->PosDesired[1];//slider_box_manual_y->getSliderVal();
					mainWindow->M_HAP_ARM->EE[2] = mainWindow->stepper_programmer->PosDesired[2];//slider_box_manual_z->getSliderVal();
					mainWindow->M_HAP_ARM->joint_vector_rad = mainWindow->M_HAP_ARM->InvKinematics(mainWindow->M_HAP_ARM->EE);
				}
			}	
		}

        cVector3d actualStepperPosSteps, actualStepperPosRad, actualEndEff;
        actualStepperPosSteps.set(mainWindow->stepper_0->actual_full_steps, mainWindow->stepper_1->actual_full_steps, mainWindow->stepper_2->actual_full_steps);
        actualStepperPosRad = mainWindow->M_HAP_ARM->convertSteps2angle(actualStepperPosSteps);
        mainWindow->stepper_0->actual_stepper_pos_double = actualStepperPosRad[0];
        mainWindow->stepper_1->actual_stepper_pos_double = actualStepperPosRad[1];
        mainWindow->stepper_2->actual_stepper_pos_double = actualStepperPosRad[2];
        actualEndEff = mainWindow->M_HAP_ARM->FwdKinematics(actualStepperPosRad);
		

		std::vector<double> actualEndEff_std_vec = std::vector<double>(3,0);
		actualEndEff_std_vec[0] = actualEndEff[0];
		actualEndEff_std_vec[1] = actualEndEff[1];
		actualEndEff_std_vec[2] = actualEndEff[2];

		mainWindow->needleBot_controller_instance->auto_line_assistoBot_posActual_vec[0] = actualEndEff[0];
		mainWindow->needleBot_controller_instance->auto_line_assistoBot_posActual_vec[1] = actualEndEff[1];
		mainWindow->needleBot_controller_instance->auto_line_assistoBot_posActual_vec[2] = actualEndEff[2];
		mainWindow->stepper_programmer->updateActualPos(actualEndEff_std_vec);

        cVector3d temp_goal_full_steps = mainWindow->M_HAP_ARM->convertAngle2steps(mainWindow->M_HAP_ARM->joint_vector_rad);
        mainWindow->stepper_0->goal_full_steps = temp_goal_full_steps[0]; //commented out on 7/27/14 @ 6:41pm while debugging use not receiving Rx messages or plotting correctly
        mainWindow->stepper_1->goal_full_steps = temp_goal_full_steps[1]; //commented out on 7/27/14 @ 6:41pm while debugging use not receiving Rx messages or plotting correctly
        mainWindow->stepper_2->goal_full_steps = temp_goal_full_steps[2]; //commented out on 7/27/14 @ 6:41pm while debugging use not receiving Rx messages or plotting correctly

		//mainWindow->stepper_0->goal_stepper_pos_double = temp_goal_full_steps[0]; //added 7/27/14 @ 6:41pm while debugging use not receiving Rx messages or plotting correctly
        //mainWindow->stepper_1->goal_stepper_pos_double = temp_goal_full_steps[1]; //added 7/27/14 @ 6:41pm while debugging use not receiving Rx messages or plotting correctly
        //mainWindow->stepper_2->goal_stepper_pos_double = temp_goal_full_steps[2]; //added 7/27/14 @ 6:41pm while debugging use not receiving Rx messages or plotting correctly

        mainWindow->M_HAP_ARM->DisplayKinematics(mainWindow->M_HAP_ARM->joint_vector_rad, mainWindow->M_HAP_ARM->EE);

        //if(mainWindow->stepper_programmer->isManualControl() == false)
        //{
			cVector3d commandedPos_needleBot, actualPos_needleBot;
			for(int i = 0; i < 3; i++)
			{
				commandedPos_needleBot[i] = mainWindow->needleBot_controller_instance->PosDesired[i];
				actualPos_needleBot[i] = mainWindow->my_DAQ->encoder_vec[i];
			}
            mainWindow->M_HAP_ARM->updateAndApplyForce(mainWindow->M_HAP_ARM->EE, actualEndEff, mainWindow->ATI_assistoBot->ATI_Force3Vec_only_smoothed, commandedPos_needleBot, actualPos_needleBot, mainWindow->ATI_needleBot->ATI_Force3Vec_only_smoothed);
        //}
    }

    printf("Exiting assistoBot thread. \n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyThread_dynamixel_servo : public QThread
{
    public:
        virtual void run();
};

void MyThread_dynamixel_servo::run()
{
	int sleep_val_ms = 5;
    while(mainWindow->start_or_end_program_flag == 1)
    {
		if(mainWindow->needleBot_controller_instance->shouldOutsideThreadsYieldControlToNeedleBotController() == 1)
		{
			mainWindow->servo_2_ethanol->goToPresetPosition(mainWindow->needleBot_controller_instance->servo_2_ethanol_PosDesired);
			mainWindow->servo_3_air->goToPresetPosition(mainWindow->needleBot_controller_instance->servo_3_air_PosDesired);
		}

		if(mainWindow->stepper_programmer->manual_control_flag == 0 && mainWindow->needleBot_controller_instance->shouldOutsideThreadsYieldControlToNeedleBotController() == 0)
		{
			mainWindow->servo_2_ethanol->goToPresetPosition(mainWindow->M_HAP_ARM->button_0_state_assistoBot);
			mainWindow->servo_3_air->goToPresetPosition(mainWindow->M_HAP_ARM->button_1_state_assistoBot);
		}

        mainWindow->servo_2_ethanol->forced_update();
		Sleep(sleep_val_ms);
        mainWindow->servo_3_air->forced_update();
		Sleep(sleep_val_ms);
        mainWindow->servo_4_restraint->forced_update();
		Sleep(sleep_val_ms);
    }

    printf("Exiting dynamixel servo thread. \n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyThread_stereo_camera_electronics_controller_QT_Tx_thread : public QThread
{
    public:
        virtual void run();
};

void MyThread_stereo_camera_electronics_controller_QT_Tx_thread::run()
{
    printf("Entering stereo_camera_electronics_controller_QT Tx thread. \n");

    while(mainWindow->start_or_end_program_flag == 1)
    {
        unsigned long Tx_sleep_time = 3;//milliseconds

        while(mainWindow->start_or_end_program_flag == 1)
        {
            mainWindow->stereo_camera_electronics_controller_QT->TxSerialMessage();
            Sleep(Tx_sleep_time); //windows only, it's usleep in linux
        }

    }

    printf("Exiting stereo_camera_electronics_controller_QT Tx thread. \n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyThread_stereo_camera_electronics_controller_QT_Rx_thread : public QThread
{
    public:
        virtual void run();
};

void MyThread_stereo_camera_electronics_controller_QT_Rx_thread::run()
{
    printf("Entering stereo_camera_electronics_controller_QT Rx thread. \n");

    bool message_done_flag = false;
    unsigned long Rx_sleep_time_ms = 3;//milliseconds
    unsigned long Rx_sleep_time_us;//microseconds
    Rx_sleep_time_us = 1000*Rx_sleep_time_ms;

     while(mainWindow->start_or_end_program_flag == 1)
     {
          mainWindow->stereo_camera_electronics_controller_QT->readMessage(message_done_flag);
          Sleep(Rx_sleep_time_ms); //windows only, it's usleep in linux //no longer needed at baud rate = 0.5MBS 11/20/2012

          if(message_done_flag == true)
          {

              message_done_flag = false;
         }
     }

    printf("Exiting stereo_camera_electronics_controller_QT Rx thread. \n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyThread_LED_board_Tx_thread : public QThread
{
    public:
        virtual void run();
};

void MyThread_LED_board_Tx_thread::run()
{
    printf("Entering LED board Tx thread. \n");

    while(mainWindow->start_or_end_program_flag == 1)
    {
        unsigned long Tx_sleep_time = 1;//milliseconds

        while(mainWindow->start_or_end_program_flag == 1)
        {
            mainWindow->LED_board_programmer->TxAllBrightnessLevels();
            Sleep(Tx_sleep_time); //windows only, it's usleep in linux
        }

    }

    printf("Exiting LED board Tx thread. \n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyThread_LED_board_Rx_thread : public QThread
{
    public:
        virtual void run();
};

void MyThread_LED_board_Rx_thread::run()
{
    printf("Entering LED board Rx thread. \n");

    bool message_done_flag = false;
    unsigned long Rx_sleep_time_ms = 1;//milliseconds
    unsigned long Rx_sleep_time_us;//microseconds
    Rx_sleep_time_us = 1000*Rx_sleep_time_ms;

     while(mainWindow->start_or_end_program_flag == 1)
     {
          mainWindow->LED_board_programmer->readMessage(message_done_flag);
          //Sleep(Rx_sleep_time_ms); //windows only, it's usleep in linux //no longer needed at baud rate = 0.5MBS 11/20/2012

          if(message_done_flag == true)
          {

              message_done_flag = false;
              //mainWindow->LED_board_RxMutex->lock();



              //mainWindow->LED_board_RxMutex->unlock();
         }
     }

    printf("Exiting LED board Rx thread. \n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyThread_composite_video_camera_assitoBot_thread : public QThread
{
    public:
        virtual void run();
};

void MyThread_composite_video_camera_assitoBot_thread::run()
{
    printf("Entering composite_video_camera_assistoBot thread. \n");

	std::string system_call_string =  "\"C:\\robot_generated_composite_video_recordings\\VLC_stream_composite_video_camera_assistoBot.bat"; 
	
	cout << system_call_string << endl;
	int system_call_return = system(system_call_string.c_str()); 

    printf("Exiting composite_video_camera_assistoBot thread. \n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyThread_composite_video_camera_ND_FPS_thread : public QThread
{
    public:
        virtual void run();
};

void MyThread_composite_video_camera_ND_FPS_thread::run()
{
    printf("Entering composite_video_camera_ND_FPS thread. \n");

	std::string system_call_string =  "\"C:\\robot_generated_composite_video_recordings\\VLC_stream_composite_video_camera_ND_FPS.bat"; 
	
	cout << system_call_string << endl;
	int system_call_return = system(system_call_string.c_str()); 

    printf("Exiting composite_video_camera_ND_FPS thread. \n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyThread_composite_video_camera_ND_bloodflash_thread : public QThread
{
    public:
        virtual void run();
};

void MyThread_composite_video_camera_ND_bloodflash_thread::run()
{
    printf("Entering composite_video_camera_ND_bloodflash thread. \n");

	std::string system_call_string =  "\"C:\\robot_generated_composite_video_recordings\\VLC_stream_composite_video_camera_ND_bloodflash.bat"; 
	
	cout << system_call_string << endl;
	int system_call_return = system(system_call_string.c_str()); 

    printf("Exiting composite_video_camera_ND_bloodflash thread. \n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyThread_test_thread : public QThread
{
    public:
        virtual void run();
};

void MyThread_test_thread::run()
{
    printf("Entering test thread. \n");

    printf("Exiting test thread. \n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    //cout << "Program entered." << endl;
	printf("Program entered. \n");

    QApplication app(argc, argv);

    mainWindow = new MainWindow();
	mainWindow->setGeometry(mainWindow->mainWindow_gui_start_x_top_monitor, mainWindow->mainWindow_gui_start_y_top_monitor, mainWindow->mainWindow_gui_single_monitor_width, mainWindow->mainWindow_gui_single_monitor_height); //starting values defined in robot_defines.h
    mainWindow->setAutoFillBackground(true);
    mainWindow->show();

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    MyThread_robot_thread robot_thread;
    robot_thread.start(QThread::TimeCriticalPriority);

    MyThread_needle_driver_Rx needle_driver_Rx_thread;
    needle_driver_Rx_thread.start(QThread::TimeCriticalPriority);

    MyThread_needle_driver_Tx needle_driver_Tx_thread;
    needle_driver_Tx_thread.start(QThread::TimeCriticalPriority);

    MyThread_stepper_Tx stepper_Tx_thread;
    stepper_Tx_thread.start(QThread::TimeCriticalPriority);

    MyThread_stepper_Rx stepper_Rx_thread;
    stepper_Rx_thread.start(QThread::TimeCriticalPriority);

    MyThread_assistoBot assistoBot_thread;
    assistoBot_thread.start(QThread::TimeCriticalPriority);

	MyThread_dynamixel_servo dynamixel_servo_thread;
    dynamixel_servo_thread.start(QThread::TimeCriticalPriority);

	MyThread_stereo_camera_electronics_controller_QT_Tx_thread stereo_camera_electronics_controller_QT_Tx_thread;
	stereo_camera_electronics_controller_QT_Tx_thread.start(QThread::TimeCriticalPriority);

	MyThread_stereo_camera_electronics_controller_QT_Rx_thread stereo_camera_electronics_controller_QT_Rx_thread;
	stereo_camera_electronics_controller_QT_Rx_thread.start(QThread::TimeCriticalPriority);

	MyThread_LED_board_Tx_thread LED_board_Tx_thread;
	LED_board_Tx_thread.start(QThread::TimeCriticalPriority);

    MyThread_LED_board_Rx_thread LED_board_Rx_thread;   
    LED_board_Rx_thread.start(QThread::TimeCriticalPriority);

	//MyThread_composite_video_camera_assitoBot_thread composite_video_camera_assitoBot_thread;   
    //composite_video_camera_assitoBot_thread.start(QThread::TimeCriticalPriority);

	//MyThread_composite_video_camera_ND_FPS_thread composite_video_camera_ND_FPS_thread;   
    //composite_video_camera_ND_FPS_thread.start(QThread::TimeCriticalPriority);

	//MyThread_composite_video_camera_ND_bloodflash_thread composite_video_camera_ND_bloodflash_thread;   
    //composite_video_camera_ND_bloodflash_thread.start(QThread::TimeCriticalPriority);

	MyThread_test_thread test_thread;   
    test_thread.start(QThread::TimeCriticalPriority);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    return app.exec(); //runs qt program NECESSARY waits until the gui the thread is finished, which is when all the windows are closed

    robot_thread.wait();
    needle_driver_Rx_thread.wait();
    needle_driver_Tx_thread.wait();
	stepper_Tx_thread.wait();
    stepper_Rx_thread.wait();
	assistoBot_thread.wait();
	dynamixel_servo_thread.wait();
	stereo_camera_electronics_controller_QT_Tx_thread.wait();
    stereo_camera_electronics_controller_QT_Rx_thread.wait();
	LED_board_Tx_thread.wait();
    LED_board_Rx_thread.wait();
	//composite_video_camera_assitoBot_thread.wait();
	//composite_video_camera_ND_FPS_thread.wait();
	//composite_video_camera_ND_bloodflash_thread.wait();
	test_thread.wait();
}
