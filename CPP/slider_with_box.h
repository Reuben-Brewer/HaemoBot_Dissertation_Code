#ifndef SLIDER_WITH_BOX_H
#define SLIDER_WITH_BOX_H

#include "print_labeled_spinbox.h"
#include "sliders.h"
#include <QMouseEvent>
#include <QMainWindow>
#include <qapplication.h>
#include <qmainwindow.h>
#include <qwt_counter.h>
#include <qtoolbar.h>
#include <qlabel.h>
#include <qlayout.h>
#include <QMutex>
#include <QSize>
#include <QThread>
#include <QWaitCondition>
#include <QApplication>
#include <QFont>
#include <QLCDNumber>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>
#include <QWidget>
#include <QDialog>
#include<QGroupBox>
#include<QFormLayout>
#include<QRadioButton>
#include<QSpinBox>
#include<QImage>
#include<QImageReader>
#include<QPixmap>
#include<QPalette>
#include<QObject>
#include <QPainter>
#include <QMessageBox>
#include <QLineEdit>
#include <QComboBox>
#include <QObject>
#include <QTGui>

class slider_with_box: public QWidget
{
    Q_OBJECT
public:
    slider_with_box(QWidget *parent, QString pre_text_in = "", int sliderType_in = 0, float slider_min_in = 0.0, float slider_max_in = 0.0, float slider_inc_in = 0.0, float slider_init_in = 0.0);
    void enable(void);
    void disable(void);
    void zero(void);
    float getSliderVal(void);
    float getSpinBoxVal(void);
    void setSliderVal(double val);
    void setSpinBoxVal(double val);
    void setVal(double val);
    void setRange(double min, double max, double inc);

    Slider *slider;
    print_labeled_spinbox *spinbox;
    float slider_init;
    double value;
    bool isPressed;

private Q_SLOTS:
    void change_spinbox(void);
    void change_slider(void);

Q_SIGNALS:
    void valueChanged(double value);

};



#endif
