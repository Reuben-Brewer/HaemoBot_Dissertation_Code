#include <stdlib.h>
#include <vector>
#include <qwt_painter.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_widget.h>
#include <qwt_legend.h>
#include <qwt_scale_draw.h>
#include <qwt_math.h>
#include "data_plot.h"
#include <qlabel.h>
#include "print_labeled_spinbox.h"
#include <qapplication.h>
#include <qframe.h>
#include <qwt_scale_map.h>
#include <qwt_symbol.h>
#include <qcolor.h>
#include <qpainter.h>
#include <qwt_math.h>
#include <qwt_scale_engine.h>
#include <qwt_plot_grid.h>
#include <qwt_text.h>
#include "print_labels.h"


using namespace std;

std::vector< QwtPlotMarker * > myMarkerVector;
QwtPlotMarker* d_mrk1;


DataPlot::DataPlot(vector<double*> data_vector_in, int auto_scale_init_in, float y_lo_init_in, float y_hi_init_in, int plot_height_in, QString plot_title_in, QString y_units_in, int plot_markers_flag_in, QWidget *parent, QMutex *my_mutex_in):
    QwtPlot(parent),
    data_vector(data_vector_in) //initializes data_vector
{
	this->auto_scale_init=auto_scale_init_in;
	this->y_lo_init=y_lo_init_in;
	this->y_hi_init=y_hi_init_in;
	this->plot_height=plot_height_in;
	this->plot_title=plot_title_in;
	this->y_units=y_units_in;
	this->plot_markers_flag=plot_markers_flag_in;
	this->my_mutex = my_mutex_in;

	max_spin_box = new print_labeled_spinbox(this, "Max: ", 0.01, 2, -1000.0, 1000.0, y_hi_init);
	QRect pos_max_spin_box = QRect(20, -4, 100, 30);   
	max_spin_box->setGeometry(pos_max_spin_box);

	min_spin_box = new print_labeled_spinbox(this, "Min: ", 0.01, 2, -1000.0, 1000.0, y_lo_init);
	QRect pos_min_spin_box = QRect(20, plot_height-20, 100, 30);   
	min_spin_box->setGeometry(pos_min_spin_box);
	
	autoscale_enable_button = new QRadioButton("Auto", this);
	QRect pos_autoscale_button = QRect(120, 0, 60, 20);   
	autoscale_enable_button->setGeometry(pos_autoscale_button);
	autoscale_enable_button->setAutoExclusive(0);
	autoscale_enable_button->setChecked(auto_scale_init);

	avg = 0;
	avg_text = new print_labels(this, &avg, "Avg: ", 3);
	QRect pos_avg_text = QRect(300, 0, 200, 12);   
	avg_text->setGeometry(pos_avg_text);
	
	cur_value = 0;
	cur_val_text = new print_labels(this, &cur_value, "Cur: ", 3);
	QRect pos_cur_val_text = QRect(300, 12, 200, 12);   
	cur_val_text->setGeometry(pos_cur_val_text);
	
	// Disable polygon clipping
    //QwtPainter::setDeviceClipping(false);

    // We don't need the cache here
    //COMMENTcanvas()->setPaintAttribute(QwtPlotCanvas::PaintCached, false);
    //COMMENTcanvas()->setPaintAttribute(QwtPlotCanvas::PaintPacked, false);

#if QT_VERSION >= 0x040000
#ifdef Q_WS_X11
    /*
       Qt::WA_PaintOnScreen is only supported for X11, but leads
       to substantial bugs with Qt 4.2.x/Windows
     */
    canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
#endif
#endif

    alignScales();
    
    //  Initialize data
    for (int i = 0; i< PLOT_SIZE; i++)
    {
        time[i] = 0.5 * i;     // time axis
    }

    // Assign a title
    setTitle(plot_title);
    //insertLegend(new QwtLegend(), QwtPlot::BottomLegend);
	QwtSymbol symbol_style;
	symbol_style.setStyle(QwtSymbol::Cross);
    
	vector<QwtPlotCurve*> curve_vector;
	curve_data = vector< vector<double> > (data_vector.size(), vector<double> (PLOT_SIZE, 0));
	
	
	vector<QColor> curve_color;
	curve_color.push_back(Qt::red);
	curve_color.push_back(Qt::blue);
	curve_color.push_back(Qt::green);
	curve_color.push_back(Qt::cyan);
	curve_color.push_back(Qt::white);
	
	
	
    for(int i = 0; i < data_vector.size(); i++)
    {
            curve_vector.push_back(new QwtPlotCurve(""));
            curve_vector[i]->attach(this);
            curve_vector[i]->setPen(QPen(curve_color[i]));
            curve_vector[i]->setRawSamples(time, &curve_data[i][0], PLOT_SIZE);
    }

/*#if 0
    //  Insert zero line at y = 0
    QwtPlotMarker *mY = new QwtPlotMarker();
    mY->setLabelAlignment(Qt::AlignRight|Qt::AlignTop);
    mY->setLineStyle(QwtPlotMarker::HLine);
    mY->setYValue(0.0);
    mY->attach(this);
#endif*/

    // Axis 
    setAxisTitle(QwtPlot::xBottom, "Time (s)");
    //setAxisScale(QwtPlot::xBottom, 0, 100);

    setAxisTitle(QwtPlot::yLeft, y_units);
   // setAxisScale(QwtPlot::yLeft, -1.5, 1.5);
    
    setAxisAutoScale(QwtPlot::yLeft);


	if(plot_markers_flag == 1)
	{
		memset(marker_vec, 0, PLOT_SIZE*sizeof(double));

		for(int j = 0; j< PLOT_SIZE; j++)
		{
				QwtPlotMarker *d_temp = new QwtPlotMarker();

				d_temp->setValue(0.0, -1000.0);
//COMMENT				d_temp->setSymbol( QwtSymbol(QwtSymbol::Diamond, QColor(Qt::green), QColor(Qt::green), QSize(10,10)));
				d_temp->attach(this);
				myMarkerVector.push_back(d_temp);
		}
	}

}


void DataPlot::alignScales()
{   
    canvas()->setFrameStyle(QFrame::Box | QFrame::Plain );
    canvas()->setLineWidth(1);
}


void DataPlot::update_data_plot(QTimerEvent *event)
{
	//printf("data plot timer event \n");


	

	
	
    // y moves from left to right:
    // Shift y array right and assign new value to y[0].
    
	my_mutex->lock();
    minVal = curve_data[0][0];
    maxVal = curve_data[0][0];
    for(int j = 0; j < data_vector.size(); j++)
    {
    	  for ( int i = PLOT_SIZE - 1; i > 0; i-- )
    		{
    		  	curve_data[j][i] = curve_data[j][i-1];
                if(curve_data[j][i] < minVal)
                    minVal = curve_data[j][i];
                if(curve_data[j][i] > maxVal)
                    maxVal = curve_data[j][i];
    		}
    	  
        curve_data[j][0] = *data_vector[j];
    }
	my_mutex->unlock();

    sum = 0;
    for(int i = 0; i < curve_data[0].size(); i++)
    {
            sum = sum + curve_data[0][i];
    }
    avg = sum/curve_data[0].size();
    avg_text->update_label(event);

    cur_value = curve_data[0][0];
    cur_val_text->update_label(event);

    autoscale_enable_button_val = autoscale_enable_button->isChecked();
    if(autoscale_enable_button_val == 1)
    {
         setAxisScale(QwtPlot::yLeft, minVal, maxVal);
    }
    else
    {
        setAxisScale(QwtPlot::yLeft, min_spin_box->my_spin_box->value(), max_spin_box->my_spin_box->value());
    }

    replot();

}


