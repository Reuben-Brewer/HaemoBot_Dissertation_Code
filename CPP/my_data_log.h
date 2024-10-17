#ifndef _MY_DATA_LOG_H
#define _MY_DATA_LOG_H 1

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <iostream>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QString>
#include <QMouseEvent>
#include "robot_defines.h"
#include "slider_with_box.h"

using namespace std;

class my_data_log: public QWidget
{
    Q_OBJECT

public:
    	my_data_log(QWidget *parent);
    	void timer_update(QTimerEvent *e);
		void save_data(void);
		void create_and_open_data_file(void);
		bool check_if_time_to_add_another_data_point(double curTime);
		void change_record_state(bool val);
		void mousePressEvent(QMouseEvent *incoming_mouse_event);

		QPushButton *record_state_button;
		QLineEdit *file_name_input_box;
		QLabel *background_box, *box_title;
		QString file_name_input, actual_file_name_to_save_with;
		vector<string> data_to_write;
		int save_data_state, new_data_file_flag, directory_made_flag;
		FILE* data_file;
		double time_last_data_point_added, desired_data_point_log_frequency;
		slider_with_box *slider_log_frequency;
		int focus_needs_to_be_restored_to_other_widgets_flag;

public Q_SLOTS:
	void change_record_state_button_toggle(void);
	void slider_log_frequency_changed(void);
	void editingFinishedResponseFunction(void);

protected:
    

private:
    

};

#endif
