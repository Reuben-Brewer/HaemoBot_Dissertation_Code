#ifndef MY_FORCE_POP_DETECTOR_H
#define MY_FORCE_POP_DETECTOR_H

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
#include "bessel_filter.h"
#include "print_labeled_spinbox.h"



class my_force_pop_detector: public QWidget
{
    Q_OBJECT

public:
	my_force_pop_detector(QWidget *parent = NULL);
    void timer_update(QTimerEvent *e);
    void update(std::vector<double> force_vector_in, double servo_loop_frequency_set_by_slider);
    void close(void);
    	
	QLabel *background_box, *box_title, *force_at_last_pop_detection_label, *force_derivative_at_last_pop_detection_label;
    std::vector<DataPlot *> plot;
    std::vector< std::vector< double * > > plot_data_vec;
    std::vector<double> ATI_FT_in, last_ATI_FT_in, ATI_FT_derivative_raw, last_ATI_FT_derivative_raw, ATI_FT_derivative_smoothed; 
	std::vector<double> ATI_FT_in_OFFSET_BY_SNAPSHOT, last_ATI_FT_in_OFFSET_BY_SNAPSHOT, ATI_FT_derivative_raw_OFFSET_BY_SNAPSHOT, last_ATI_FT_derivative_raw_OFFSET_BY_SNAPSHOT, ATI_FT_in_SNAPSHOT;
	bessel_filter *force_derivative_bessel_filter;
	QPushButton *reset_pop_detection_latch_button, *force_offset_snapshot_button;
	print_labeled_spinbox *force_pop_detection_threshold_spinbox, *force_derivative_pop_detection_threshold_spinbox;
	double force_pop_detection_threshold, force_derivative_pop_detection_threshold;
    QMutex *plot_mutex;
	double pop_detection_state, pop_detection_state_latched;
	double force_at_last_pop_detection, force_derivative_at_last_pop_detection;



public Q_SLOTS:
    void reset_pop_detection_latch(void);
	void take_force_snapshot(void);
};



#endif





