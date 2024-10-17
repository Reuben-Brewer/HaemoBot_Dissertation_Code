#ifndef _PRINT_LABELED_SPINBOX_H
#define _PRINT_LABELED_SPINBOX_H 1

#include <iostream>
#include <qwidget.h>
#include <qwt_slider.h>
#include <qlabel.h>
#include <qwt_plot.h>
#include<QSpinBox>
#include <math.h>
#include <qapplication.h>
#include <qlabel.h>
#include <qlayout.h>
#include <qwt_slider.h>
#include <qwt_scale_engine.h>
#include <qwt_scale_map.h>
#include "print_labels.h"
#include <QKeyEvent>

using namespace std;

class QLabel;
class QWidget;

class print_labeled_spinbox: public QWidget
{
    Q_OBJECT

public:
    print_labeled_spinbox(QWidget* = NULL, QString pre_text_in = "", float single_step_in = 0, int decimal_precision_in = 0, float min_range_in = 0, float max_range_in = 0, float init_value_in = 0);
	double getValue(void);
	void enable(void);
	void disable(void);
	void update_label(QTimerEvent *e);
	void setValue(double val);
	void keyPressEvent(QKeyEvent *incoming_key_event);

	QLabel *d_label;
	float single_step, min_range, max_range, init_value;
	QString pre_text, dummy;
	QDoubleSpinBox *my_spin_box;
	int decimal_precision;	
	double current_value;

public Q_SLOTS:
	void editingFinishedResponseFunction(double val);
};

#endif
