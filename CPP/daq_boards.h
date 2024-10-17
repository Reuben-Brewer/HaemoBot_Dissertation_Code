#ifndef DAQ_BOARDS_H
#define DAQ_BOARDS_H

#define bessel_order 2

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <NIDAQmx.h>
#include "robot_defines.h"
#include <QWidget>
#include <QMutex>;
#include <QLabel>
#include "bessel_filter.h"

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

class DAQ_board: public QWidget
{
	Q_OBJECT

    public:
        DAQ_board(QWidget *parent = NULL);
		void timer_update(QTimerEvent *e);
        void init_to_zero(void);
        void close(void);
        void write_ao(void);
		void read_di(void);
        void write_do(int32 write_value);
        void zero_ao(void);
        void update_encoders(void);
        double update_global_clock(void);
        void compute_motor_torque_to_write(void);
        double voltage_saturation(double desired_V, double max_V, double min_V);
        void update_ATI(void);
		void toggle_do_DMT(void);
		void set_do_DMT(int val);


		int DI_state_X_mag_0, DI_state_X_mag_1, DI_state_X_slot_0, DI_state_X_slot_1, DI_state_X_slot_home, DI_state_X_enable, DI_state_DMT; 
		int DI_state_Y_mag_0, DI_state_Y_mag_1, DI_state_Y_slot_0, DI_state_Y_slot_1, DI_state_Y_slot_home, DI_state_Y_enable;
		int DI_state_Z_mag_0, DI_state_Z_mag_1, DI_state_Z_slot_0, DI_state_Z_slot_1, DI_state_Z_enable;
		int DI_state_YAW_slot_home, DI_state_YAW_enable;
		int DI_state_pitch_slot_home, DI_state_pitch_enable;
		int DI_state_roll_enable;
		int DI_state_catheter_insertion_enable;
		int DI_state_catheter_CC_enable;

		QLabel *background_box, *box_title;
		QLabel *DI_state_X_mag_0_label, *DI_state_X_mag_1_label, *DI_state_X_slot_0_label, *DI_state_X_slot_1_label, *DI_state_X_slot_home_label, *DI_state_X_enable_label, *DI_state_DMT_label; 
		QLabel *DI_state_Y_mag_0_label, *DI_state_Y_mag_1_label, *DI_state_Y_slot_0_label, *DI_state_Y_slot_1_label, *DI_state_Y_slot_home_label, *DI_state_Y_enable_label;
		QLabel *DI_state_Z_mag_0_label, *DI_state_Z_mag_1_label, *DI_state_Z_slot_0_label, *DI_state_Z_slot_1_label, *DI_state_Z_enable_label;
		QLabel *DI_state_YAW_slot_home_label, *DI_state_YAW_enable_label;
		QLabel *DI_state_pitch_slot_home_label, *DI_state_pitch_enable_label;
		QLabel *DI_state_roll_enable_label;
		QLabel *DI_state_catheter_insertion_enable_label;
		QLabel *DI_state_catheter_CC_enable_label;
		QLabel *DAQ_encoder_0_label, *DAQ_encoder_1_label, *DAQ_encoder_2_label, *DAQ_encoder_3_label, *DAQ_encoder_4_label, *DAQ_encoder_5_label, *DAQ_encoder_6_label, *DAQ_encoder_7_label;
		QLabel *servo_loop_frequency_set_by_slider_label;

		int servo_loop_software_DMT_state;
        double current_global_time;
		float64 last_global_time;
        float64 current_clock_ticks, last_clock_ticks;
        std::vector<double> ATI_assistoBot_raw_vec, ATI_needleBot_raw_vec;
		double needle_driver_stringpot_voltage_raw, needle_driver_stringpot_position, needle_driver_stringpot_conversion_constant_B, needle_driver_stringpot_conversion_constant_M;
		std::vector<double> encoder_vec, encoder_INCREMENTAL_vec, encoder_dot_vec, encoder_offset_at_calibration_vec, last_encoder_vec, last_encoder_dot_vec;
		std::vector<double> motor_torque_to_write_AO, analog_voltage_to_write_AO_vec, current_to_write_AO_vec, voltage_for_max_cont_current_from_copely_vec, kt_vec, motor_V_over_A_vec;
		std::vector<double> motor_enable_software_vec, motor_enable_hardware_vec;
		std::vector<int> home_flag_vec, neg_slot_flag_vec, neg_mag_flag_vec, pos_slot_flag_vec, pos_mag_flag_vec;

        QMutex ATI_mini_40_mutex;

        TaskHandle taskHandle_ao;
        TaskHandle taskHandle_AI;
        TaskHandle taskHandle_di_6723, taskHandle_di_6224;
        TaskHandle taskHandle_do;
        TaskHandle taskHandleEncoder_0;
        TaskHandle taskHandleEncoder_1;
        TaskHandle taskHandleEncoder_2;
        TaskHandle taskHandleEncoder_3;
        TaskHandle taskHandleEncoder_4;
        TaskHandle taskHandleEncoder_5;
        TaskHandle taskHandleEncoder_6;
        TaskHandle taskHandleEncoder_7;
        TaskHandle taskHandle_clock_out;
        TaskHandle taskHandle_clock_in;

		std::vector<double> velocity_bessel_smoothed_vec;
		bessel_filter *velocity_bessel_filter;
		
		double servo_loop_frequency_set_by_slider;
};

#endif


