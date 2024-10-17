#include "ATI_force_sensor.h"




ATI_force_sensor::ATI_force_sensor(QWidget *parent, QString calibration_filepath_in, QString sensor_name_in):
QWidget(parent)
{
    calibration_filepath = calibration_filepath_in;
	sensor_name = sensor_name_in;

	/////////////////////////background_box must be the first GUI object so that it doesn't cover over other objects!!!
	background_box = new QLabel("",this);
	background_box->setGeometry(0,0,1280,380);
	background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(rgb(200,255,200))}");

    ATI_FT_raw = std::vector<double>(6,0);

	Bias_being_computed_block_flag = 0;
	ATI_FT_raw_BiasVoltage_queue_vec = std::vector<std::queue<double>>(6, std::queue<double>());

    ATI_FT_smoothed = std::vector<double>(6,0);
    ATI_FT_dot = std::vector<double>(6,0);
	ATI_Force3Vec_only_smoothed = std::vector<double>(3,0);

	force_sensor_has_been_biased_at_startup_flag = 0;

    plot_mutex = new QMutex;
	bias_being_computed_mutex = new QMutex;

    int plot_height = 175;
	int plot_height_inc = 10;
    int plot_width = 370;
	int plot_width_inc = 10;
	int plot_start_x = 5;
	int plot_start_y = 5;

    std::vector<double *> temp_0;
    temp_0.push_back(&ATI_FT_raw[0]);
    temp_0.push_back(&ATI_FT_smoothed[0]);
    plot_data_vec.push_back(temp_0);
    plot.push_back(new DataPlot(plot_data_vec[0], 1, -8.0, 8.0, plot_height, QString("Fx (N)"), "N", 0, this, plot_mutex));
    plot[0]->setGeometry(QRect(plot_start_x + 0*(plot_width + plot_width_inc), plot_start_y + 0*(plot_height + plot_height_inc), plot_width, plot_height));

    std::vector<double *> temp_1;
    temp_1.push_back(&ATI_FT_raw[1]);
    temp_1.push_back(&ATI_FT_smoothed[1]);
    plot_data_vec.push_back(temp_1);
    plot.push_back(new DataPlot(plot_data_vec[1], 1, -8.0, 8.0, plot_height, QString("Fy (N)"), "N", 0, this, plot_mutex));
    plot[1]->setGeometry(QRect(plot_start_x + 1*(plot_width + plot_width_inc), plot_start_y + 0*(plot_height + plot_height_inc), plot_width, plot_height));

    std::vector<double *> temp_2;
    temp_2.push_back(&ATI_FT_raw[2]);
    temp_2.push_back(&ATI_FT_smoothed[2]);
    plot_data_vec.push_back(temp_2);
    plot.push_back(new DataPlot(plot_data_vec[2], 1, -15.0, 0.0, plot_height, QString("Fz (N)"), "N", 0, this, plot_mutex));
    plot[2]->setGeometry(QRect(plot_start_x + 2*(plot_width + plot_width_inc), plot_start_y + 0*(plot_height + plot_height_inc), plot_width, plot_height));

	std::vector<double *> temp_3;
    temp_3.push_back(&ATI_FT_raw[3]);
    temp_3.push_back(&ATI_FT_smoothed[3]);
    plot_data_vec.push_back(temp_3);
    plot.push_back(new DataPlot(plot_data_vec[3], 1, -8.0, 8.0, plot_height, QString("Tx (NM)"), "NM", 0, this, plot_mutex));
    plot[3]->setGeometry(QRect(plot_start_x + 0*(plot_width + plot_width_inc), plot_start_y + 1*(plot_height + plot_height_inc), plot_width, plot_height));

    std::vector<double *> temp_4;
    temp_4.push_back(&ATI_FT_raw[4]);
    temp_4.push_back(&ATI_FT_smoothed[4]);
    plot_data_vec.push_back(temp_4);
    plot.push_back(new DataPlot(plot_data_vec[4], 1, -8.0, 8.0, plot_height, QString("Ty (NM)"), "NM", 0, this, plot_mutex));
    plot[4]->setGeometry(QRect(plot_start_x + 1*(plot_width + plot_width_inc), plot_start_y + 1*(plot_height + plot_height_inc), plot_width, plot_height));

    std::vector<double *> temp_5;
    temp_5.push_back(&ATI_FT_raw[5]);
    temp_5.push_back(&ATI_FT_smoothed[5]);
    plot_data_vec.push_back(temp_5);
    plot.push_back(new DataPlot(plot_data_vec[5], 1, -15.0, 0.0, plot_height, QString("Tz (NM)"), "NM", 0, this, plot_mutex));
    plot[5]->setGeometry(QRect(plot_start_x + 2*(plot_width + plot_width_inc), plot_start_y + 1*(plot_height + plot_height_inc), plot_width, plot_height));

    sensor_name_label = new QLabel(sensor_name, this);
    sensor_name_label->setGeometry(QRect(plot_start_x + 3*(plot_width + plot_width_inc),5, 200, 20));

    biasVoltageButton = new QPushButton("Bias Voltage", this);
    QRect pos_biasVoltageButton = QRect(plot_start_x + 3*(plot_width + plot_width_inc),35,120,20);
    biasVoltageButton->setGeometry(pos_biasVoltageButton);
    connect(biasVoltageButton, SIGNAL(clicked()), this, SLOT(BiasVoltage()));

    load_calibration(0);

    SetForceUnits(cal,"N"); // Set force units.
    SetTorqueUnits(cal,"N-m"); // Set torque units.

	////////////////////////////////////////////////////Zero these variables to eliminate incorrect starting blips in the data.
	for(int i = 0; i < bessel_order+1; i++)
	{
		for(int j = 0; j < 6; j++)
		{
			FT_raw[i][j] = 0; 
			FT_filtered[i][j] = 0; 
		}
	}
	////////////////////////////////////////////////////
}


void ATI_force_sensor::update(std::vector<double> raw_DAQ_voltages)
{
	bias_being_computed_mutex->lock();
    float voltages[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float lambda_FT_dot = 1/(1+100*2*pi_define*(1/pid_f));
    float last_FT_dot[6];
    long double temp_vec[bessel_order+1];
    //static long double FT_raw[bessel_order+1][6]; //Must be static so that we can have a history that remains between function calls
    //static long double FT_filtered[bessel_order+1][6]; //Must be static so that we can have a history that remains between function calls
    long double a[bessel_order+1] = {0.0000,   -1.9485,    0.9494}; //original robot filter, except the 1 was a zero,  omega = 30, f = 4.7746Hz
    long double b[bessel_order+1] = {0.2193,    0.4385,    0.2193}; //original robot filter,  omega = 30, f = 4.7746Hz
	//long double a[bessel_order+1] = {1.0000,   -1.0973,    0.3376}; //100Hz
    //long double b[bessel_order+1] = {0.0601,    0.1202,    0.0601}; //100Hz
	//long double a[bessel_order+1] = {0.0000,   -1.6911,    0.7215}; //100Hz
    //long double b[bessel_order+1] = {0.0076,    0.0152,    0.0076}; //100Hz


    

    //Don't modify the voltages AT ALL prior to the ConvertToFT call because it will screw up the transformation!
    /////////////////////////////////////////////////////////////
    for(int m = 0; m <= 5; m++)
    {
		voltages[m] = raw_DAQ_voltages[m]; //These are the raw voltages from the strain gauges before being converted to forces via the calibration matrix multiplication.
			
			/////////////////////////////////////////////////////////////MUST BIAS WITH RAW VOLTAGES BEFORE APPLYING THE ConvertToFT FUNCTION (NOT AFTER)!!!
			if(Bias_being_computed_block_flag == 0)
			{
				if(ATI_FT_raw_BiasVoltage_queue_vec[m].size() < ATI_bias_minimum_sample_num_for_computation)
				{
					ATI_FT_raw_BiasVoltage_queue_vec[m].push(voltages[m]); //adds new data point without deleting the oldest value.
				}
				else
				{
					ATI_FT_raw_BiasVoltage_queue_vec[m].push(voltages[m]); //adds new data point and then deletes the oldest value to keep the size manageable
					ATI_FT_raw_BiasVoltage_queue_vec[m].pop();
				}
			}
			/////////////////////////////////////////////////////////////
    }

    float temp_FT_raw[6];
    ConvertToFT(cal, voltages, temp_FT_raw);
    //printf("%f  %f  %f  %f  %f  %f \r",temp_FT_raw[0], temp_FT_raw[1], temp_FT_raw[2], temp_FT_raw[3], temp_FT_raw[4], temp_FT_raw[5]);
    /////////////////////////////////////////////////////////////

    for(int axis_counter = 0; axis_counter <= 5; axis_counter++)
    {
            /////////////////////////////////////////////////////////////
            //temp_vec FOR DEBUGGING ONLY
            //check that raw data all makes it in
            //printf("%llf %llf %llf %llf %llf %llf \r", FT_raw[0][0], FT_raw[0][1], FT_raw[0][2], FT_raw[0][3], FT_raw[0][4], FT_raw[0][5]);
            //make copy of pre-shifted vector so that we can check that shifting works later.
            float temp_vec[6] = {0, 0, 0, 0, 0, 0};
            for(int counter = bessel_order; counter >= 0; counter--)
            {
                    temp_vec[counter] = FT_raw[counter][axis_counter];
            }
            /////////////////////////////////////////////////////////////

            //shift data
            for(int memory_counter = bessel_order; memory_counter >= 1; memory_counter--)
            {
                    FT_raw[memory_counter][axis_counter] = FT_raw[memory_counter-1][axis_counter];
                    FT_filtered[memory_counter][axis_counter] = FT_filtered[memory_counter-1][axis_counter];
            }

            //Update newest data point
            FT_raw[0][axis_counter] = temp_FT_raw[axis_counter];

            //Check that we're actually shifting...all values but last should be zero.
           // printf("%f %f %f %f %f %f %f \r", FT_raw[6][axis_counter]-temp_vec[5], FT_raw[5][axis_counter]-temp_vec[4], FT_raw[4][axis_counter]-temp_vec[3], FT_raw[3][axis_counter]-temp_vec[2], FT_raw[2][axis_counter]-temp_vec[1], FT_raw[1][axis_counter]-temp_vec[0], FT_raw[6][2]);

            FT_filtered[0][axis_counter] = 0;
            for(int k = 0; k <= bessel_order; k++)
            {
                    FT_filtered[0][axis_counter] = FT_filtered[0][axis_counter] + (1e-3)*b[k]*FT_raw[k][axis_counter] - a[k]*FT_filtered[k][axis_counter];
            }

            ATI_FT_smoothed[axis_counter] = FT_filtered[0][axis_counter];
            ATI_FT_raw[axis_counter] = temp_FT_raw[axis_counter];
            ATI_FT_dot[axis_counter] = lambda_FT_dot*last_FT_dot[axis_counter] + (1-lambda_FT_dot)*(FT_raw[0][axis_counter]-FT_raw[1][axis_counter])*pid_f;

            last_FT_dot[axis_counter] = ATI_FT_dot[axis_counter];
    }

	ATI_Force3Vec_only_smoothed[0] = ATI_FT_smoothed[0];
	ATI_Force3Vec_only_smoothed[1] = ATI_FT_smoothed[1];
	ATI_Force3Vec_only_smoothed[2] = ATI_FT_smoothed[2];
	bias_being_computed_mutex->unlock();
}

void ATI_force_sensor::close(void)
{
    destroy_calibration();
}

void ATI_force_sensor::timer_update(QTimerEvent *event)
{
    for(int i = 0; i < plot.size(); i++)
    {
        plot[i]->update_data_plot(event);
    }
}

void ATI_force_sensor::BiasVoltage(void)
{
	bias_being_computed_mutex->lock();
	Bias_being_computed_block_flag = 1;

	std::vector<double> sum_vec = std::vector<double>(6,0);
	std::vector<double> N_vec = std::vector<double>(6,0);
	std::vector<double> average_vec = std::vector<double>(6,0);
	float biasVoltages[6];

	if(ATI_FT_raw_BiasVoltage_queue_vec[0].size() >= ATI_bias_minimum_sample_num_for_computation) //set in robot_defines.h
	{
		for(int i = 0; i < 6; i++)
		{
			while(ATI_FT_raw_BiasVoltage_queue_vec[i].size() > 0)
			{
				sum_vec[i] = sum_vec[i] + ATI_FT_raw_BiasVoltage_queue_vec[i].front();
				ATI_FT_raw_BiasVoltage_queue_vec[i].pop();
				N_vec[i] = N_vec[i] + 1;
			}
			average_vec[i] = sum_vec[i]/N_vec[i];
			biasVoltages[i] = average_vec[i];
		}
		cout << "Computed ATI bias values with " << N_vec[0] << " samples, and the queue is now size " << ATI_FT_raw_BiasVoltage_queue_vec[0].size() << endl;
		cout << "voltFx avg: " << average_vec[0] << " voltFy avg: " << average_vec[1] << " voltFz avg: " << average_vec[2] << " voltTx avg: " << average_vec[3] << " voltTy avg: " << average_vec[4] << " voltTz avg: " << average_vec[5] << endl;
	}
	else
	{
		cout << "Not enough samples to compute ATI bias values. You have only " << ATI_FT_raw_BiasVoltage_queue_vec[0].size() << " samples, and you need " << ATI_bias_minimum_sample_num_for_computation << " samples." << endl;
	}

	destroy_calibration();
	load_calibration(0);
	Bias(cal, biasVoltages); //ATI function call to apply bias voltages
	Bias_being_computed_block_flag = 0;
	bias_being_computed_mutex->unlock();
}

std::vector<double> ATI_force_sensor::getForce3_smoothed(void)
{
    std::vector<double> force3;
    force3.push_back(ATI_FT_smoothed[0]);
    force3.push_back(ATI_FT_smoothed[1]);
    force3.push_back(ATI_FT_smoothed[2]);
    return force3;
}

std::vector<double> ATI_force_sensor::getTorque3_smoothed(void)
{
    std::vector<double> torque3;
    torque3.push_back(ATI_FT_smoothed[3]);
    torque3.push_back(ATI_FT_smoothed[4]);
    torque3.push_back(ATI_FT_smoothed[5]);
    return torque3;
}

std::vector<double> ATI_force_sensor::getFT6_smoothed(void)
{
    std::vector<double> FT6;
    FT6.push_back(ATI_FT_smoothed[0]);
    FT6.push_back(ATI_FT_smoothed[1]);
    FT6.push_back(ATI_FT_smoothed[2]);
    FT6.push_back(ATI_FT_smoothed[3]);
    FT6.push_back(ATI_FT_smoothed[4]);
    FT6.push_back(ATI_FT_smoothed[5]);
    return FT6;
}

void ATI_force_sensor::load_calibration(int print_calibration_flag)
{
	QByteArray calibration_filepath_QByteArray_format = calibration_filepath.toLatin1();
	char *calibration_filepath_charstar_format = calibration_filepath_QByteArray_format.data();

	cal = createCalibration(calibration_filepath_charstar_format,1);
	if(print_calibration_flag == 1)
	{
		printCalInfo(cal);
	}
}

void ATI_force_sensor::destroy_calibration(void)
{
	destroyCalibration(cal);
}
