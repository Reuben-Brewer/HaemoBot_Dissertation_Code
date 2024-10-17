#include "daq_boards.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
DAQ_board::DAQ_board(QWidget *parent):
QWidget(parent)
{
	/////////////////////////background_box must be the first GUI object so that it doesn't cover over other objects!!!
	background_box = new QLabel("",this);
	background_box->setGeometry(0,0,250,1440-30);
	background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(rgb(200,255,200))}");

	box_title = new QLabel("DAQ Cards",this);
	box_title->setGeometry(10,10,150,30);
	box_title->setFont(QFont("Times", 12, QFont::Bold));

	ATI_assistoBot_raw_vec = std::vector<double>(6, 0);
	ATI_needleBot_raw_vec = std::vector<double>(6, 0);
	needle_driver_stringpot_voltage_raw = 0;
	needle_driver_stringpot_position = 0;

	encoder_vec = std::vector<double>(8, 0);
	encoder_INCREMENTAL_vec = std::vector<double>(8, 0);
	encoder_offset_at_calibration_vec = std::vector<double>(8, 0);
	encoder_dot_vec = std::vector<double>(8, 0);
	last_encoder_vec = std::vector<double>(8, 0);
	last_encoder_dot_vec = std::vector<double>(8, 0);
	velocity_bessel_smoothed_vec = std::vector<double>(8,0);
	
	motor_torque_to_write_AO = std::vector<double>(7, 0);
	analog_voltage_to_write_AO_vec = std::vector<double>(7, 0);
	current_to_write_AO_vec = std::vector<double>(7, 0);
	voltage_for_max_cont_current_from_copely_vec = std::vector<double>(7, 0);
	kt_vec = std::vector<double>(7, 0);
	motor_V_over_A_vec = std::vector<double>(7, 0);
	motor_enable_software_vec = std::vector<double>(7, 0);
	motor_enable_hardware_vec = std::vector<double>(7, 0);
	home_flag_vec = std::vector<int>(7, -1);
	neg_slot_flag_vec = std::vector<int>(7, -1);
	neg_mag_flag_vec = std::vector<int>(7, -1);
	pos_slot_flag_vec = std::vector<int>(7, -1);
	pos_mag_flag_vec = std::vector<int>(7, -1);

	int DAQ_label_start_x = 5;
	int DAQ_label_start_y = 55;
	int DAQ_label_width = 250;
	int DAQ_label_height = 20;
	int DAQ_label_height_inc = 5;

    DI_state_X_mag_0_label = new QLabel("", this);
    DI_state_X_mag_0_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 0*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_X_mag_1_label = new QLabel("", this);
    DI_state_X_mag_1_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 1*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_X_slot_0_label = new QLabel("", this);
    DI_state_X_slot_0_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 2*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_X_slot_1_label = new QLabel("", this);
    DI_state_X_slot_1_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 3*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_X_slot_home_label = new QLabel("", this);
    DI_state_X_slot_home_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 4*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_X_enable_label = new QLabel("", this);
    DI_state_X_enable_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 5*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_DMT_label = new QLabel("", this);
    DI_state_DMT_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 6*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

    DI_state_Y_mag_0_label = new QLabel("", this);
    DI_state_Y_mag_0_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 7*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_Y_mag_1_label = new QLabel("", this);
    DI_state_Y_mag_1_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 8*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_Y_slot_0_label = new QLabel("", this);
    DI_state_Y_slot_0_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 9*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_Y_slot_1_label = new QLabel("", this);
    DI_state_Y_slot_1_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 10*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_Y_slot_home_label = new QLabel("", this);
    DI_state_Y_slot_home_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 11*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_Y_enable_label = new QLabel("", this);
    DI_state_Y_enable_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 12*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

    DI_state_Z_mag_0_label = new QLabel("", this);
    DI_state_Z_mag_0_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 13*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_Z_mag_1_label = new QLabel("", this);
    DI_state_Z_mag_1_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 14*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_Z_slot_0_label = new QLabel("", this);
    DI_state_Z_slot_0_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 15*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_Z_slot_1_label = new QLabel("", this);
    DI_state_Z_slot_1_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 16*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_Z_enable_label = new QLabel("", this);
    DI_state_Z_enable_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 17*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_YAW_slot_home_label = new QLabel("", this);
    DI_state_YAW_slot_home_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 18*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));
	
	DI_state_YAW_enable_label = new QLabel("", this);
    DI_state_YAW_enable_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 19*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_pitch_slot_home_label = new QLabel("", this);
    DI_state_pitch_slot_home_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 20*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));
	
	DI_state_pitch_enable_label = new QLabel("", this);
    DI_state_pitch_enable_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 21*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_roll_enable_label = new QLabel("", this);
    DI_state_roll_enable_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 22*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_catheter_insertion_enable_label = new QLabel("", this);
    DI_state_catheter_insertion_enable_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 23*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DI_state_catheter_CC_enable_label = new QLabel("", this);
    DI_state_catheter_CC_enable_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 24*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DAQ_encoder_0_label = new QLabel("", this);
    DAQ_encoder_0_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 25*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DAQ_encoder_1_label = new QLabel("", this);
    DAQ_encoder_1_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 26*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DAQ_encoder_2_label = new QLabel("", this);
    DAQ_encoder_2_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 27*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DAQ_encoder_3_label = new QLabel("", this);
    DAQ_encoder_3_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 28*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DAQ_encoder_4_label = new QLabel("", this);
    DAQ_encoder_4_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 29*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DAQ_encoder_5_label = new QLabel("", this);
    DAQ_encoder_5_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 30*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DAQ_encoder_6_label = new QLabel("", this);
    DAQ_encoder_6_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 31*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	DAQ_encoder_7_label = new QLabel("", this);
    DAQ_encoder_7_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 32*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));

	servo_loop_frequency_set_by_slider_label = new QLabel("", this);
    servo_loop_frequency_set_by_slider_label->setGeometry(QRect(DAQ_label_start_x,DAQ_label_start_y + 33*(DAQ_label_height + DAQ_label_height_inc), DAQ_label_width, DAQ_label_height));



	///////////////////////////////////////////////////////
	velocity_bessel_filter = new bessel_filter(this, 8); 
	///////////////////////////////////////////////////////

	init_to_zero();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void DAQ_board::init_to_zero(void)
{		
    printf("Initializing DAQ boards. \n");

	//PCI6224 = DEV1
	//PCI6723 = DEV2
	//PCI6602 = DEV3

	DAQmxCreateTask("AO",&taskHandle_ao);
    DAQmxCreateAOVoltageChan(taskHandle_ao,"Dev2/ao0:6","",-10.0,10.0,DAQmx_Val_Volts,"");
    DAQmxStartTask(taskHandle_ao);
    zero_ao();

    DAQmxCreateTask("Digital in 6723",&taskHandle_di_6723);
    int result_create_DI_channel_6723 = DAQmxCreateDIChan(taskHandle_di_6723,"Dev2/port0/line0:6","",DAQmx_Val_ChanForAllLines); //Enable signal out
    DAQmxStartTask(taskHandle_di_6723);  

	DAQmxCreateTask("Digital in 6224",&taskHandle_di_6224);
    int result_create_DI_channel_6224 = DAQmxCreateDIChan(taskHandle_di_6224,"Dev1/port0/line0:7, Dev1/port1/line0:2, Dev1/port1/line4:7, Dev1/port2/line0:2","",DAQmx_Val_ChanForAllLines); //Enable signal out
    DAQmxStartTask(taskHandle_di_6224);  

    DAQmxCreateTask("Digital out",&taskHandle_do); //For toggling a line to show the servo loop rate on O-scope.
    DAQmxCreateDOChan(taskHandle_do,"Dev2/port0/line7","",DAQmx_Val_ChanForAllLines);
    DAQmxStartTask(taskHandle_do);

	DAQmxCreateTask("AI",&taskHandle_AI);
    DAQmxCreateAIVoltageChan(taskHandle_AI,"Dev1/ai0:5, Dev1/ai16:21","AI",DAQmx_Val_Diff ,-10.0,10.0,DAQmx_Val_Volts,NULL); //ATI Force sensors
	DAQmxCreateAIVoltageChan(taskHandle_AI,"Dev1/ai6","AI",DAQmx_Val_RSE ,-10.0,10.0,DAQmx_Val_Volts,NULL); //Needle Driver Stringpot
    DAQmxStartTask(taskHandle_AI);

    DAQmxCreateTask("Encoder 0",&taskHandleEncoder_0);
    DAQmxCreateCIAngEncoderChan(taskHandleEncoder_0,"Dev3/ctr0", "channel 0", DAQmx_Val_X4, 0, 0.0, DAQmx_Val_AHighBHigh, DAQmx_Val_Degrees, 1.0, 0.0,"");
    DAQmxStartTask(taskHandleEncoder_0);

    DAQmxCreateTask("Encoder 1",&taskHandleEncoder_1);
    DAQmxCreateCIAngEncoderChan(taskHandleEncoder_1,"Dev3/ctr1", "channel 1", DAQmx_Val_X4, 0, 0.0, DAQmx_Val_AHighBHigh, DAQmx_Val_Degrees, 1.0, 0.0,"");
    DAQmxStartTask(taskHandleEncoder_1);

    DAQmxCreateTask("Encoder 2",&taskHandleEncoder_2);
    DAQmxCreateCILinEncoderChan(taskHandleEncoder_2,"Dev3/ctr2", "channel 2", DAQmx_Val_X4, 0, 0.0, DAQmx_Val_AHighBHigh, DAQmx_Val_Meters, 1.0, 0.0,"");
    DAQmxStartTask(taskHandleEncoder_2);

    DAQmxCreateTask("Encoder 3",&taskHandleEncoder_3);
    DAQmxCreateCIAngEncoderChan(taskHandleEncoder_3,"Dev3/ctr3", "channel 3", DAQmx_Val_X4, 0, 0.0, DAQmx_Val_AHighBHigh, DAQmx_Val_Degrees, 1.0, 0.0,"");
    DAQmxStartTask(taskHandleEncoder_3);

    DAQmxCreateTask("Encoder 4",&taskHandleEncoder_4);
    DAQmxCreateCIAngEncoderChan(taskHandleEncoder_4,"Dev3/ctr4", "channel 4", DAQmx_Val_X4, 0, 0.0, DAQmx_Val_AHighBHigh, DAQmx_Val_Degrees, 1.0, 0.0,"");
    DAQmxStartTask(taskHandleEncoder_4);

    DAQmxCreateTask("Encoder 5",&taskHandleEncoder_5);
    DAQmxCreateCIAngEncoderChan(taskHandleEncoder_5,"Dev3/ctr5", "channel 5", DAQmx_Val_X4, 0, 0.0, DAQmx_Val_AHighBHigh, DAQmx_Val_Degrees, 1.0, 0.0,"");
    DAQmxStartTask(taskHandleEncoder_5);

	DAQmxCreateTask("Encoder 6",&taskHandleEncoder_6);
    DAQmxCreateCIAngEncoderChan(taskHandleEncoder_6,"Dev3/ctr6", "channel 6", DAQmx_Val_X4, 0, 0.0, DAQmx_Val_AHighBHigh, DAQmx_Val_Degrees, 1.0, 0.0,"");
    DAQmxStartTask(taskHandleEncoder_6);

	DAQmxCreateTask("Encoder 7",&taskHandleEncoder_7);
    DAQmxCreateCIAngEncoderChan(taskHandleEncoder_7,"Dev3/ctr7", "channel 7", DAQmx_Val_X4, 0, 0.0, DAQmx_Val_AHighBHigh, DAQmx_Val_Degrees, 1.0, 0.0,"");
    DAQmxStartTask(taskHandleEncoder_7);

    DAQmxCreateTask("Clock Out",&taskHandle_clock_out);
    DAQmxCreateCOPulseChanFreq(taskHandle_clock_out,"Dev2/ctr0","",DAQmx_Val_Hz,DAQmx_Val_Low,0.0,sample_f,0.5);
    DAQmxCfgImplicitTiming(taskHandle_clock_out,DAQmx_Val_ContSamps,1000);
    DAQmxStartTask(taskHandle_clock_out);

    DAQmxCreateTask("Clock In",&taskHandle_clock_in);
    DAQmxCreateCICountEdgesChan(taskHandle_clock_in,"Dev2/ctr1", "", DAQmx_Val_Rising , 0, DAQmx_Val_CountUp);
    DAQmxStartTask(taskHandle_clock_in);

    current_global_time = 0;
	current_clock_ticks = 0;
	last_clock_ticks = 0;
	servo_loop_software_DMT_state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void DAQ_board::close(void)
{
    zero_ao();
	set_do_DMT(0);

	DAQmxStopTask(taskHandle_ao);
    DAQmxClearTask(taskHandle_ao);

    DAQmxStopTask(taskHandleEncoder_0);
    DAQmxClearTask(taskHandleEncoder_0);

    DAQmxStopTask(taskHandleEncoder_1);
    DAQmxClearTask(taskHandleEncoder_1);

    DAQmxStopTask(taskHandleEncoder_2);
    DAQmxClearTask(taskHandleEncoder_2);

    DAQmxStopTask(taskHandleEncoder_3);
    DAQmxClearTask(taskHandleEncoder_3);

    DAQmxStopTask(taskHandleEncoder_4);
    DAQmxClearTask(taskHandleEncoder_4);

    DAQmxStopTask(taskHandleEncoder_5);
    DAQmxClearTask(taskHandleEncoder_5);

    DAQmxStopTask(taskHandleEncoder_6);
    DAQmxClearTask(taskHandleEncoder_6);

	DAQmxStopTask(taskHandleEncoder_7);
    DAQmxClearTask(taskHandleEncoder_7);

    DAQmxStopTask(taskHandle_di_6723);
    DAQmxClearTask(taskHandle_di_6723);

	DAQmxStopTask(taskHandle_di_6224);
    DAQmxClearTask(taskHandle_di_6224);

    DAQmxStopTask(taskHandle_do);
    DAQmxClearTask(taskHandle_do);

    DAQmxStopTask(taskHandle_clock_out);
    DAQmxClearTask(taskHandle_clock_out);

    DAQmxStopTask(taskHandle_clock_in);
    DAQmxClearTask(taskHandle_clock_in);

    DAQmxStopTask(taskHandle_AI);
    DAQmxClearTask(taskHandle_AI);

    return;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void DAQ_board::write_ao(void)
{

    float64 data_ao_f64[7];
	
	for(int i = 0; i < 7; i++)
	{
		data_ao_f64[i] = analog_voltage_to_write_AO_vec[i]*motor_enable_hardware_vec[i]*motor_enable_software_vec[i];
	}
	
	DAQmxWriteAnalogF64(taskHandle_ao,1,1,10.0,DAQmx_Val_GroupByChannel,data_ao_f64,NULL,NULL);
    return;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void DAQ_board::read_di(void)
{
    uInt8 data_di_6723[7], data_di_6224[18];
	int32 result_6723, read_sampsPerChanRead_6723, read_numBytesPerSamp_6723;
	int32 result_6224, read_sampsPerChanRead_6224, read_numBytesPerSamp_6224;

	//int32  DAQmxReadDigitalLines (TaskHandle taskHandle, int32 numSampsPerChan, float64 timeout,       bool32 fillMode,      uInt8 readArray[],  uInt32 arraySizeInBytes,   int32 *sampsPerChanRead,      int32 *numBytesPerSamp, bool32 *reserved);
	result_6723 = DAQmxReadDigitalLines (taskHandle_di_6723,           DAQmx_Val_Auto,               1,    DAQmx_Val_GroupByChannel,     data_di_6723,                  7,     &read_sampsPerChanRead_6723,  &read_numBytesPerSamp_6723,             NULL);
	
	//int32  DAQmxReadDigitalLines (TaskHandle taskHandle, int32 numSampsPerChan, float64 timeout,       bool32 fillMode,      uInt8 readArray[],  uInt32 arraySizeInBytes,   int32 *sampsPerChanRead,      int32 *numBytesPerSamp, bool32 *reserved);
	result_6224 = DAQmxReadDigitalLines (taskHandle_di_6224,           DAQmx_Val_Auto,               1,    DAQmx_Val_GroupByChannel,     data_di_6224,                  18,     &read_sampsPerChanRead_6224,  &read_numBytesPerSamp_6224,             NULL);

	//6723 port0, line 0 = x mag -
	DI_state_X_mag_0 = data_di_6723[0];
	neg_mag_flag_vec[0] = DI_state_X_mag_0;

	//6723 port0, line 1 = x mag + 
	DI_state_X_mag_1 = data_di_6723[1];
	pos_mag_flag_vec[0] = DI_state_X_mag_1;

	//6723 port0, line 2 = x slot 0 -
	DI_state_X_slot_0 = data_di_6723[2];
	neg_slot_flag_vec[0] = DI_state_X_slot_0;

	//6723 port0, line 3 = x slot 1 +
	DI_state_X_slot_1 = data_di_6723[3];
	pos_slot_flag_vec[0] = DI_state_X_slot_1;

	//6723 port0, line 4 = x slot home
	DI_state_X_slot_home = data_di_6723[4];
	home_flag_vec[0] = DI_state_X_slot_home;

	//6723 port0, line 5 = x enable
	DI_state_X_enable = data_di_6723[5];
	motor_enable_hardware_vec[0] = DI_state_X_enable;

	//6723 port0, line 6 = x DMT
	DI_state_DMT = data_di_6723[6];

	//6224 port0, line 0 = y mag 0 -
	DI_state_Y_mag_0 = data_di_6224[0];
	neg_mag_flag_vec[1] = DI_state_Y_mag_0;

	//6224 port0, line 1 = y mag 1 +
	DI_state_Y_mag_1 = data_di_6224[1];
	pos_mag_flag_vec[1] = DI_state_Y_mag_1;

	//6224 port0, line 2 = y slot 0 -
	DI_state_Y_slot_0 = data_di_6224[2];
	neg_slot_flag_vec[1] = DI_state_Y_slot_0;

	//6224 port0, line 3 = y slot 1 +
	DI_state_Y_slot_1 = data_di_6224[3];
	pos_slot_flag_vec[1] = DI_state_Y_slot_1;

	//6224 port0, line 4 = y slot home
	DI_state_Y_slot_home = DI_state_Y_slot_1; //data_di_6224[4]; OVERRIDING THE HOME SENSOR SO THAT WE USE THE BACK END SLOT SENSOR FOR HOMING
	home_flag_vec[1] = DI_state_Y_slot_home;

	//6224 port0, line 5 = y enable
	DI_state_Y_enable = data_di_6224[5];
	motor_enable_hardware_vec[1] = DI_state_Y_enable;

	//6224 port0, line 6 = z mag 0 -
	DI_state_Z_mag_0 = data_di_6224[6];
	neg_mag_flag_vec[2] = DI_state_Z_mag_0;

	//6224 port0, line 7 = z mag 1 +
	DI_state_Z_mag_1 = data_di_6224[7];
	pos_mag_flag_vec[2] = DI_state_Z_mag_1;

	//6224 port1, line 0 = z slot 0-
	DI_state_Z_slot_0 = data_di_6224[8];
	neg_slot_flag_vec[2] = DI_state_Z_slot_0;

	//6224 port1, line 1 = z slot 1+
	DI_state_Z_slot_1 = data_di_6224[9];
	home_flag_vec[2] = DI_state_Z_slot_1;
	pos_slot_flag_vec[2] = DI_state_Z_slot_1;

	//6224 port1, line 2 = z enable
	DI_state_Z_enable = data_di_6224[10];
	motor_enable_hardware_vec[2] = DI_state_Z_enable;

	//6224 port1, line 4 = yaw slot home
	DI_state_YAW_slot_home = data_di_6224[11];
	home_flag_vec[3] = DI_state_YAW_slot_home;

	//6224 port 1, line 5 = yaw enable
	DI_state_YAW_enable = data_di_6224[12];
	motor_enable_hardware_vec[3] = DI_state_YAW_enable;

	//6224 port1, line 6 = pitch slot home
	DI_state_pitch_slot_home = data_di_6224[13];
	home_flag_vec[4] = DI_state_pitch_slot_home;

	//6224 port1 line 7 = pitch enable
	DI_state_pitch_enable = data_di_6224[14];
	motor_enable_hardware_vec[4] = DI_state_pitch_enable;

	//6224 port 2, line 0 = roll enable
	DI_state_roll_enable = data_di_6224[15];
	motor_enable_hardware_vec[5] = DI_state_roll_enable;

	//6224 port 2, line 1 = catheter insertion CI enable
	DI_state_catheter_insertion_enable = data_di_6224[16];
	motor_enable_hardware_vec[6] = DI_state_catheter_insertion_enable;

	//6224 port 2, line 2 = catheter CC enable
	DI_state_catheter_CC_enable = data_di_6224[17];

    return;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void DAQ_board::write_do(int32 write_value)
{
    uInt8 data_do[1];
    data_do[0] = write_value;
    DAQmxWriteDigitalLines(taskHandle_do,1,1,10.0,DAQmx_Val_GroupByChannel, data_do,NULL,NULL);
    return;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void DAQ_board::update_ATI(void)
{
    int32       error=0;
    char        errBuff[2048]={'\0'};

    int32 read_ai;
    float64 ATI_ai_voltage_float64[13];

    DAQmxErrChk(DAQmxReadAnalogF64(taskHandle_AI, 1, 10.0, DAQmx_Val_GroupByChannel, ATI_ai_voltage_float64, 13, &read_ai, NULL));
	
    for(int i = 0; i <= 5; i++)
    {
        ATI_assistoBot_raw_vec[i] = ATI_ai_voltage_float64[i];
    }
    
	for(int j = 0; j <= 5; j++)
    {
		ATI_needleBot_raw_vec[j] = ATI_ai_voltage_float64[6+j];
    }

	needle_driver_stringpot_voltage_raw = ATI_ai_voltage_float64[12];

	needle_driver_stringpot_conversion_constant_M = rd_needle_driver_stringpot_PositionMax/(rd_needle_driver_stringpot_Vmax - rd_needle_driver_stringpot_Vmin);
	needle_driver_stringpot_conversion_constant_B = -1.0*rd_needle_driver_stringpot_PositionMax*rd_needle_driver_stringpot_Vmin/(rd_needle_driver_stringpot_Vmax - rd_needle_driver_stringpot_Vmin);
	
	needle_driver_stringpot_position = needle_driver_stringpot_conversion_constant_M*needle_driver_stringpot_voltage_raw + needle_driver_stringpot_conversion_constant_B;


    Error:
        if( DAQmxFailed(error) )
        {
                DAQmxGetExtendedErrorInfo(errBuff,2048);
                printf("DAQmx Error: %s\n",errBuff);
        }

    return;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void DAQ_board::zero_ao(void)
{
	for(int i = 0; i < 7; i++)
	{
        analog_voltage_to_write_AO_vec[i] = 0;
	}
	write_ao();
	return;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void DAQ_board::update_encoders(void)
{
		int32 read_encoder;
        static uInt32 temp_encoder_vec_uint32[8];
        float temp_encoder_vec_unwrapped[8];
		
		DAQmxReadCounterU32(taskHandleEncoder_0, 1, 1.0, &temp_encoder_vec_uint32[0], 1, &read_encoder, 0);
		DAQmxReadCounterU32(taskHandleEncoder_1, 1, 1.0, &temp_encoder_vec_uint32[1], 1, &read_encoder, 0);
		DAQmxReadCounterU32(taskHandleEncoder_2, 1, 1.0, &temp_encoder_vec_uint32[2], 1, &read_encoder, 0);
		DAQmxReadCounterU32(taskHandleEncoder_3, 1, 1.0, &temp_encoder_vec_uint32[3], 1, &read_encoder, 0);
		DAQmxReadCounterU32(taskHandleEncoder_4, 1, 1.0, &temp_encoder_vec_uint32[4], 1, &read_encoder, 0);
		DAQmxReadCounterU32(taskHandleEncoder_5, 1, 1.0, &temp_encoder_vec_uint32[5], 1, &read_encoder, 0);
		DAQmxReadCounterU32(taskHandleEncoder_6, 1, 1.0, &temp_encoder_vec_uint32[6], 1, &read_encoder, 0);
		DAQmxReadCounterU32(taskHandleEncoder_7, 1, 1.0, &temp_encoder_vec_uint32[7], 1, &read_encoder, 0);

               //printf("%u %u %u %u %u %f\r", temp_encoder_vec_uint32[0], temp_encoder_vec_uint32[1], temp_encoder_vec_uint32[2], temp_encoder_vec_uint32[3], temp_encoder_vec_uint32[4], current_global_time);
						
        for(int joint_counter = 0; joint_counter <= 7; joint_counter++)
		{
			if(temp_encoder_vec_uint32[joint_counter] > 100000000)
			{
				temp_encoder_vec_unwrapped[joint_counter] = (float(temp_encoder_vec_uint32[joint_counter])- 4294967295.0);
			}
			else
			{
				temp_encoder_vec_unwrapped[joint_counter] = float(temp_encoder_vec_uint32[joint_counter]);
			}
		}
		
                encoder_vec[0] = temp_encoder_vec_unwrapped[0]*(10.0/1.0)*(1.0/(encoder_cpr_0*4.0)); //Linear distance in mm
                encoder_vec[1] = -1.0*temp_encoder_vec_unwrapped[1]*(10.0/1.0)*(1.0/(encoder_cpr_1*4.0)); //Linear distance in mm //The -1.0* front multiplication is required to get the Y-axis in the correct direction.
                encoder_vec[2] = -1.0*temp_encoder_vec_unwrapped[2]*(10.0/1.0)*(1.0/(encoder_cpr_2*4.0)); //Linear distance in mm
                encoder_vec[3] = -1.0*temp_encoder_vec_unwrapped[3]*(14.0/72.0)*(360.0/1.0)*(1.0/(encoder_cpr_3*4.0)); //num_degrees_yaw = (14/72)gear_ratio*(360deg/1rev)*num_ticks*(1rev/(1250ticks*4))
                encoder_vec[4] = temp_encoder_vec_unwrapped[4]*(16.0/190.0)*(360.0/1.0)*(1.0/(encoder_cpr_4*4.0)); //num_degrees_pitch = (16/190)gear_ratio*(360deg/1rev)*num_ticks*(1rev/(1250ticks*4))
				encoder_vec[5] = temp_encoder_vec_unwrapped[5]*(10.0/36.0)*(1.0/4.0)*(360.0/1.0)*(1.0/(encoder_cpr_5*4.0)); //num_degrees_roll = (10/36)gear_ratio*(1/4)gear_ratio*(360deg/1rev)*num_ticks*(1rev/(256ticks*4))
				encoder_vec[6] = temp_encoder_vec_unwrapped[6]*(9.525/1.0)*(1.0/(encoder_cpr_6*4.0)); //num_mm_cath_ins = (9.525mm/rev)gear_ratio*num_ticks*(1rev/(256ticks*4)) //USE THIS LINE TO SET THE CATH INS DOF POSITION SOLEY FROM THE BUSTED ENCODER
				encoder_vec[7] = temp_encoder_vec_unwrapped[7]*(1.0/1.0)*(1.0/(encoder_cpr_7*4.0)); //NEED TO SET

				for(int q = 0; q < 7; q++)
				{
					encoder_INCREMENTAL_vec[q] = encoder_vec[q];
				}

				//OVERIDE THE INCREMENTAL ENCODER ON THE CATH INS DOF
				encoder_vec[6] = needle_driver_stringpot_position; //USE THIS LINE TO SET THE CATH INS DOF POSITION SOLEY FROM THE STRINGPOT

				for(int i = 0; i < 7; i++)
				{
					encoder_vec[i] = encoder_vec[i] - encoder_offset_at_calibration_vec[i];

					encoder_dot_vec[i] = (encoder_vec[i]-last_encoder_vec[i])*(1.0/(1.0/servo_loop_frequency_set_by_slider));
					last_encoder_vec[i] = encoder_vec[i];
					last_encoder_dot_vec[i] = encoder_dot_vec[i];
				}

				velocity_bessel_smoothed_vec = velocity_bessel_filter->update(encoder_dot_vec);

				velocity_bessel_smoothed_vec[5] = encoder_dot_vec[5];//REMOVE THIS HACK THAT BYPASSES THE SMOOTHING FUNCTION

		return;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
double DAQ_board::update_global_clock(void)
{
    DAQmxReadCounterScalarF64(taskHandle_clock_in, 10.0, &current_clock_ticks, 0);
    current_global_time = current_global_time + (current_clock_ticks - last_clock_ticks)/sample_f;
    last_clock_ticks = current_clock_ticks;
	return current_global_time;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void DAQ_board::compute_motor_torque_to_write(void)
{
/*	double analog_voltage_to_write_AO_vec_temp[7];

    for(int i = 0; i < 7; i++)
    {
            current_to_write_AO_vec[i] = motor_torque_to_write_AO[i]/kt_vec[i];
            analog_voltage_to_write_AO_vec_temp[i] = current_to_write_AO_vec[i]*motor_V_over_A_vec[i];
            analog_voltage_to_write_AO_vec[i] = voltage_saturation(analog_voltage_to_write_AO_vec_temp[i], voltage_for_max_cont_current_from_copely_vec[i], -1*voltage_for_max_cont_current_from_copely_vec[i]);
    }
	analog_voltage_to_write_AO_vec[0] = 1; /////////REMOVE/////////// 
	analog_voltage_to_write_AO_vec[1] = 2; /////////REMOVE/////////// 
	analog_voltage_to_write_AO_vec[2] = 3; /////////REMOVE/////////// 
	analog_voltage_to_write_AO_vec[3] = 4; /////////REMOVE/////////// 
	analog_voltage_to_write_AO_vec[4] = 5; /////////REMOVE/////////// 
	analog_voltage_to_write_AO_vec[5] = 6; /////////REMOVE/////////// 
	analog_voltage_to_write_AO_vec[6] = 7; /////////REMOVE/////////// */
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
double DAQ_board::voltage_saturation(double desired_V, double max_V, double min_V)
{
	static double desired_saturated_V;
    desired_saturated_V = desired_V;

    if(desired_V > max_V)
    {
        desired_saturated_V = max_V;
    }
    else if(desired_V < min_V)
    {
		desired_saturated_V = min_V;
    }

    return desired_saturated_V;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void DAQ_board::timer_update(QTimerEvent *event)
{
	DI_state_X_mag_0_label->setText("DI_state_X_mag_0: " + QString::number(DI_state_X_mag_0));
	DI_state_X_mag_1_label->setText("DI_state_X_mag_1: " + QString::number(DI_state_X_mag_1));
	DI_state_X_slot_0_label->setText("DI_state_X_slot_0: " + QString::number(DI_state_X_slot_0));
	DI_state_X_slot_1_label->setText("DI_state_X_slot_1: " + QString::number(DI_state_X_slot_1));
	DI_state_X_slot_home_label->setText("DI_state_X_slot_home: " + QString::number(DI_state_X_slot_home));
	DI_state_X_enable_label->setText("DI_state_X_enable: " + QString::number(DI_state_X_enable));
	DI_state_DMT_label->setText("DI_state_DMT: " + QString::number(DI_state_DMT));
	DI_state_Y_mag_0_label->setText("DI_state_Y_mag_0: " + QString::number(DI_state_Y_mag_0));
	DI_state_Y_mag_1_label->setText("DI_state_Y_mag_1: " + QString::number(DI_state_Y_mag_1));
	DI_state_Y_slot_0_label->setText("DI_state_Y_slot_0: " + QString::number(DI_state_Y_slot_0));
	DI_state_Y_slot_1_label->setText("DI_state_Y_slot_1: " + QString::number(DI_state_Y_slot_1));
	DI_state_Y_slot_home_label->setText("DI_state_Y_slot_home: " + QString::number(DI_state_Y_slot_home));
	DI_state_Y_enable_label->setText("DI_state_Y_enable: " + QString::number(DI_state_Y_enable));
	DI_state_Z_mag_0_label->setText("DI_state_Z_mag_0: " + QString::number(DI_state_Z_mag_0));
	DI_state_Z_mag_1_label->setText("DI_state_Z_mag_1: " + QString::number(DI_state_Z_mag_1));
	DI_state_Z_slot_0_label->setText("DI_state_Z_slot_0: " + QString::number(DI_state_Z_slot_0));
	DI_state_Z_slot_1_label->setText("DI_state_Z_slot_1: " + QString::number(DI_state_Z_slot_1));
	DI_state_Z_enable_label->setText("DI_state_Z_enable: " + QString::number(DI_state_Z_enable));
	DI_state_YAW_slot_home_label->setText("DI_state_YAW_slot_home: " + QString::number(DI_state_YAW_slot_home));
	DI_state_YAW_enable_label->setText("DI_state_YAW_enable: " + QString::number(DI_state_YAW_enable));
	DI_state_pitch_slot_home_label->setText("DI_state_pitch_slot_home: " + QString::number(DI_state_pitch_slot_home));
	DI_state_pitch_enable_label->setText("DI_state_pitch_enable: " + QString::number(DI_state_pitch_enable));
	DI_state_roll_enable_label->setText("DI_state_roll_enable: " + QString::number(DI_state_roll_enable));
	DI_state_catheter_insertion_enable_label->setText("DI_state_catheter_insertion_enable: " + QString::number(DI_state_catheter_insertion_enable));
	DI_state_catheter_CC_enable_label->setText("DI_state_catheter_CC_enable: " + QString::number(DI_state_catheter_CC_enable));

	DAQ_encoder_0_label->setText("DAQ_encoder_0: " + QString::number(encoder_vec[0]));
	DAQ_encoder_1_label->setText("DAQ_encoder_1: " + QString::number(encoder_vec[1]));
	DAQ_encoder_2_label->setText("DAQ_encoder_2: " + QString::number(encoder_vec[2]));
	DAQ_encoder_3_label->setText("DAQ_encoder_3: " + QString::number(encoder_vec[3]));
	DAQ_encoder_4_label->setText("DAQ_encoder_4: " + QString::number(encoder_vec[4]));
	DAQ_encoder_5_label->setText("DAQ_encoder_5: " + QString::number(encoder_vec[5]));
	DAQ_encoder_6_label->setText("DAQ_encoder_6: " + QString::number(encoder_vec[6]));
	DAQ_encoder_7_label->setText("DAQ_encoder_7: " + QString::number(encoder_vec[7]));

	servo_loop_frequency_set_by_slider_label->setText("Servo F: " + QString::number(servo_loop_frequency_set_by_slider));
}

void DAQ_board::toggle_do_DMT(void)
{
	if(servo_loop_software_DMT_state == 0)
	{
		set_do_DMT(1);
	}
	else if(servo_loop_software_DMT_state == 1)
	{
		set_do_DMT(0);
	}
}

void DAQ_board::set_do_DMT(int val)
{
	if(val == 0)
	{
		servo_loop_software_DMT_state = 0;
	}
	else if(val == 1)
	{
		servo_loop_software_DMT_state = 1;
	}

	write_do(servo_loop_software_DMT_state);
}


