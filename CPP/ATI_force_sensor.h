#ifndef ATI_FORCE_SENSOR_H
#define ATI_FORCE_SENSOR_H

#define bessel_order 2

#include <iostream>
#include "slider_with_box.h"
#include <string>
#include <vector>
#include <queue>
#include "data_plot.h"
#include <stdio.h>
#include <stdlib.h>
#include "ftconfig.h"
#include <QMutex>
#include <QPushButton>
#include <QLabel>
#include "robot_defines.h"



class ATI_force_sensor: public QWidget
{
    Q_OBJECT

public:
	ATI_force_sensor(QWidget *parent = NULL, QString calibration_filepath_in = NULL, QString sensor_name_in = NULL);
    void timer_update(QTimerEvent *e);
    void update(std::vector<double> raw_DAQ_voltages);
    void close(void);
    std::vector<double> getForce3_smoothed(void);
    std::vector<double> getTorque3_smoothed(void);
    std::vector<double> getFT6_smoothed(void);
	void load_calibration(int print_calibration_flag);
	void destroy_calibration(void);
	
	QString calibration_filepath, sensor_name;
    QPushButton *biasVoltageButton;
	QLabel *sensor_name_label, *background_box;
    std::vector<DataPlot *> plot;
    std::vector< std::vector< double * > > plot_data_vec;
    Calibration *cal;	// struct containing calibration information
    std::vector<double> ATI_FT_raw, ATI_FT_smoothed, ATI_FT_dot, ATI_Force3Vec_only_smoothed;
	std::vector<std::queue<double>> ATI_FT_raw_BiasVoltage_queue_vec;
    QMutex *plot_mutex, *bias_being_computed_mutex;
	int Bias_being_computed_block_flag;
	bool force_sensor_has_been_biased_at_startup_flag;

	long double FT_raw[bessel_order+1][6]; //Must be static so that we can have a history that remains between function calls
    long double FT_filtered[bessel_order+1][6]; //Must be static so that we can have a history that remains between function calls

public Q_SLOTS:
    void BiasVoltage(void);
};



#endif





