#ifndef BESSEL_FILTER_H
#define BESSEL_FILTER_H

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



class bessel_filter: public QWidget
{
    Q_OBJECT

public:
	bessel_filter(QWidget *parent = NULL, int length_of_vector_to_be_smoothed_in = 1);
    void timer_update(QTimerEvent *e);
    std::vector<double> update(std::vector<double> vec);
    void close(void);

    std::vector<double> signal_out_smoothed, signal_in_raw;
	std::vector<std::queue<double>> ATI_FT_raw_BiasVoltage_queue_vec;
    QMutex *bias_being_computed_mutex;
	int Bias_being_computed_block_flag;
	int length_of_vector_to_be_smoothed;

	std::vector< std::vector<double> > FT_raw_vec, FT_filtered_vec;

public Q_SLOTS:

};



#endif





