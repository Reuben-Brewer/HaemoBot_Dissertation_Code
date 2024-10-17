#define sample_f 10000.0 //Global clock frequency for the DAQ card. Was originally 1000000.0 in first robot. When you use the original value, the encoders stop updating.
#define pi_define 3.141592654
#define ATI_bias_minimum_sample_num_for_computation 10

#define rd_needle_driver_stringpot_PositionMax 42.24 //mm
#define rd_needle_driver_stringpot_Vmax 4.342 //volts
#define rd_needle_driver_stringpot_Vmin 0.478 //volts

//encoder CPRS
#define encoder_cpr_0 1250.0 //NEED TO SET
#define encoder_cpr_1 1250.0 //NEED TO SET
#define encoder_cpr_2 1250.0 //NEED TO SET
#define encoder_cpr_3 1250.0 //NEED TO SET
#define encoder_cpr_4 1250.0 //NEED TO SET
#define encoder_cpr_5 256.0 //NEED TO SET
#define encoder_cpr_6 256.0 //NEED TO SET
#define encoder_cpr_7 1250.0 //NEED TO SET

#define rd_position_offset_for_needleBot_to_assistoBot_calibration_0 -13 //X
#define rd_position_offset_for_needleBot_to_assistoBot_calibration_1 -108 //Y
#define rd_position_offset_for_needleBot_to_assistoBot_calibration_2 -26.3 //Z
#define rd_position_offset_for_needleBot_to_assistoBot_calibration_3 0 //YAW
#define rd_position_offset_for_needleBot_to_assistoBot_calibration_4 0 //PITCH
#define rd_position_offset_for_needleBot_to_assistoBot_calibration_5 0 //ROLL
#define rd_position_offset_for_needleBot_to_assistoBot_calibration_6 0 //CATH INS

#define pid_f 1000.0

#define rd_Kt_0 238 //Units are Nmm/Apk for RE-50-370357
#define rd_Kt_1 238 //Units are Nmm/Apk for RE-50-370357
#define rd_Kt_2 238 //Units are Nmm/Apk for RE-50-370357
#define rd_Kt_3 53.8 //Units are Nmm/Apk for RE-30-310009
#define rd_Kt_4 53.8 //Units are Nmm/Apk for RE-30-310009
#define rd_Kt_5 9.55 //Units are Nmm/Apk for RE-10 
#define rd_Kt_6 9.55 //Units are Nmm/Apk for RE-10 

#define rd_Kp_start_val_0 100 //x
#define rd_Kp_start_val_1 100 //y
#define rd_Kp_start_val_2 100 //z
#define rd_Kp_start_val_3 50 //yaw
#define rd_Kp_start_val_4 30 //pitch
#define rd_Kp_start_val_5 0.5 //roll //was 0.25 7:30pm 9/14/2014
#define rd_Kp_start_val_6 0.3 //cath ins

#define rd_Kv_PIDcontroller_start_val_0 1 //x
#define rd_Kv_PIDcontroller_start_val_1 1 //y
#define rd_Kv_PIDcontroller_start_val_2 1 //z
#define rd_Kv_PIDcontroller_start_val_3 5 //yaw
#define rd_Kv_PIDcontroller_start_val_4 1 //pitch
#define rd_Kv_PIDcontroller_start_val_5 0.002 //roll, was .0005 7:30pm 9/14/2014
#define rd_Kv_PIDcontroller_start_val_6 0.02 //cath ins

#define rd_Kv_velController_start_val_0 18 //x
#define rd_Kv_velController_start_val_1 15 //y
#define rd_Kv_velController_start_val_2 29 //z
#define rd_Kv_velController_start_val_3 15 //yaw
#define rd_Kv_velController_start_val_4 5.5 //pitch
#define rd_Kv_velController_start_val_5 0.0005 //roll
#define rd_Kv_velController_start_val_6 0.02 //cath ins

#define rd_Ki_start_val_0 0 //x
#define rd_Ki_start_val_1 0 //y
#define rd_Ki_start_val_2 0 //z
#define rd_Ki_start_val_3 0 //yaw
#define rd_Ki_start_val_4 0 //pitch
#define rd_Ki_start_val_5 0 //roll
#define rd_Ki_start_val_6 0.002 //cath ins

#define rd_motor_V_over_A_0 0.582072176949942 //Units are volts/amps for RE-50-370357, 17.18A = 10V
#define rd_motor_V_over_A_1 0.582072176949942 //Units are volts/amps for RE-50-370357, 17.18A = 10V
#define rd_motor_V_over_A_2 0.582072176949942 //Units are volts/amps for RE-50-370357, 17.18A = 10V
#define rd_motor_V_over_A_3 0.527426160337553 //Units are volts/amps for RE-30-310009, 18.96A = 10V
#define rd_motor_V_over_A_4 0.527426160337553 //Units are volts/amps for RE-30-310009, 18.96A = 10V
#define rd_motor_V_over_A_5 5.0 //Units are volts/amps for RE-10, 2A = 10V
#define rd_motor_V_over_A_6 5.0 //Units are volts/amps for RE-10, 2A = 10V

#define gear_ratio_0 10 //10mm lead for LX4510
#define gear_ratio_1 10 //10mm lead for LX4510
#define gear_ratio_2 10 //10mm lead for LX4510
#define gear_ratio_3 1 //NEED TO SET
#define gear_ratio_4 1 //NEED TO SET
#define gear_ratio_5 1 //NEED TO SET
#define gear_ratio_6 1 //NEED TO SET
#define gear_ratio_7 1 //NEED TO SET

#define R0 195.5

#define rd_voltage_for_max_cont_current_from_copely_0 0.942956926658906 //1.62A cont max @ 0.5820721769499418 volts/amp  for RE-50-370357
#define rd_voltage_for_max_cont_current_from_copely_1 0.942956926658906 //1.62A cont max @ 0.5820721769499418 volts/amp  for RE-50-370357
#define rd_voltage_for_max_cont_current_from_copely_2 0.942956926658906 //1.62A cont max @ 0.5820721769499418 volts/amp  for RE-50-370357
#define rd_voltage_for_max_cont_current_from_copely_3 0.864978902953586 //1.64A cont max @ 0.5274261603375527 volts/amp  for RE-30-310009
#define rd_voltage_for_max_cont_current_from_copely_4 0.864978902953586 //1.64A cont max @ 0.52742616033755271.62* volts/amp  for RE-30-310009
#define rd_voltage_for_max_cont_current_from_copely_5 0.405 //0.081A cont max @ 5.0 volts/amp for RE-10
#define rd_voltage_for_max_cont_current_from_copely_6 0.405 //0.081A cont max @ 5.0 volts/amp for RE-10

#define rd_vel_filter_omega_0 1000.0 //15.0 //NEED TO SET
#define rd_vel_filter_omega_1 1000.0 //15.0 //NEED TO SET
#define rd_vel_filter_omega_2 1000.0 //15.0 //NEED TO SET
#define rd_vel_filter_omega_3 15.0 //50.0 //NEED TO SET
#define rd_vel_filter_omega_4 50.0 //15.0 //NEED TO SET
#define rd_vel_filter_omega_5 800.0 //set for roll
#define rd_vel_filter_omega_6 50.0 //50.0 //NEED TO SET
#define rd_vel_filter_omega_7 50.0 //50.0 //NEED TO SET

#define rd_calibration_speed_for_starting_ZERO_on_home_sensor_0 -10 //x
#define rd_calibration_speed_for_starting_ZERO_on_home_sensor_1 10 //y
#define rd_calibration_speed_for_starting_ZERO_on_home_sensor_2 10 //z
#define rd_calibration_speed_for_starting_ZERO_on_home_sensor_3 10 //yaw
#define rd_calibration_speed_for_starting_ZERO_on_home_sensor_4 10 //pitch
#define rd_calibration_speed_for_starting_ZERO_on_home_sensor_5 -100 //roll
#define rd_calibration_speed_for_starting_ZERO_on_home_sensor_6 -10 //cath ins

#define rd_calibration_speed_for_starting_ONE_on_home_sensor_0 10 //x
#define rd_calibration_speed_for_starting_ONE_on_home_sensor_1 -10 //y
#define rd_calibration_speed_for_starting_ONE_on_home_sensor_2 -10 //z
#define rd_calibration_speed_for_starting_ONE_on_home_sensor_3 -10 //yaw
#define rd_calibration_speed_for_starting_ONE_on_home_sensor_4 -10 //pitch
#define rd_calibration_speed_for_starting_ONE_on_home_sensor_5 100 //roll
#define rd_calibration_speed_for_starting_ONE_on_home_sensor_6 10 //cath ins

#define rd_control_mode_start_0 1 //x
#define rd_control_mode_start_1 1 //y
#define rd_control_mode_start_2 1 //z
#define rd_control_mode_start_3 1 //yaw
#define rd_control_mode_start_4 1 //pitch
#define rd_control_mode_start_5 2 //roll
#define rd_control_mode_start_6 1 //cath ins

#define rd_percent_effort_of_max_continuous_init_0 1.0 //X
#define rd_percent_effort_of_max_continuous_init_1 1.0 //Y
#define rd_percent_effort_of_max_continuous_init_2 1.0 //Z
#define rd_percent_effort_of_max_continuous_init_3 1.0 //YAW
#define rd_percent_effort_of_max_continuous_init_4 1.5 //PITCH
#define rd_percent_effort_of_max_continuous_init_5 1.0 //ROLL
#define rd_percent_effort_of_max_continuous_init_6 1.0 //CATH INS



