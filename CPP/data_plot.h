#ifndef _DATA_PLOT_H
#define _DATA_PLOT_H 1

#include <vector>
#include <qwt_plot.h>
#include<QRadioButton>
#include<QSpinBox>
#include<QLineEdit>
#include <qlabel.h>
#include<QDoubleSpinBox>
#include "print_labeled_spinbox.h"
#include "print_labels.h"
#include <QMutex>

using namespace std;

const int PLOT_SIZE = 201;      // 0 to 200

class QLabel;

class DataPlot : public QwtPlot
{
    Q_OBJECT

public:
        DataPlot(vector<double*> data_vector_in, int auto_scale_init_in = 0,  float y_lo_init_in = 0.0, float y_hi_init_in = 0.0, int plot_height_in = 0, QString plot_title_in  = "", QString y_units_in  = "", int plot_markers_flag_in = 0, QWidget* = NULL, QMutex* my_mutex_in = NULL);
	void update_data_plot(QTimerEvent *event);
	int plot_height, plot_markers_flag, auto_scale_init;
	QRadioButton *autoscale_enable_button;
	bool autoscale_enable_button_val;
	float max_spin_box_val, min_spin_box_val;
	QString plot_title, y_units;
	print_labeled_spinbox *max_spin_box, *min_spin_box;
	void alignScales();
	float sum, avg, y_hi_init, y_lo_init, cur_value;
	double time[PLOT_SIZE];
	double marker_vec[PLOT_SIZE];
	vector< vector<double> > curve_data;
        vector<double*> data_vector;
	print_labels *avg_text, *cur_val_text;
	QMutex *my_mutex;	
    double minVal, maxVal;
};

#endif
