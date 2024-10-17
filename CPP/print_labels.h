#ifndef _PRINT_LABELS_H
#define _PRINT_LABELS_H 1

#include <qwidget.h>
#include <qwt_slider.h>
#include <qlabel.h>
#include <qwt_plot.h>

class QLabel;
class QWidget;

class print_labels: public QWidget
{
    Q_OBJECT

public:
        print_labels(QWidget* = NULL, float* label_num_in = 0, QString pre_text_in = "", int decimal_precision_in = 0);
	void update_label(QTimerEvent *e);
	QLabel *d_label;
	float* label_num;
	QString pre_text, dummy;
	int decimal_precision;
};

#endif
