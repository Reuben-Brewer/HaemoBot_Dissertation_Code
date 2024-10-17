#include <math.h>
#include <qapplication.h>
#include <qlabel.h>
#include <qlayout.h>
#include <qwt_slider.h>
#include <qwt_scale_engine.h>
#include <qwt_scale_map.h>
#include "sliders.h"
#include <QFont>

class Layout: public QBoxLayout
{
public:
    Layout(Qt::Orientation o, QWidget *parent = NULL):
#if QT_VERSION < 0x040000
        QBoxLayout(parent, QBoxLayout::LeftToRight)
#else
        QBoxLayout(QBoxLayout::LeftToRight, parent)
#endif
    {
        if ( o == Qt::Vertical )
            setDirection(QBoxLayout::TopToBottom);

        setSpacing(20);
        setMargin(0);
    }
};

Slider::Slider(QWidget *parent, QString pre_text_in, int sliderType_in, float slider_min_in, float slider_max_in, float slider_inc_in, float slider_init_in):
    QWidget(parent)
{
	this->pre_text = pre_text_in;
	this->sliderType = sliderType_in;
	this->slider_min = slider_min_in;
	this->slider_max = slider_max_in;
	this->slider_inc = slider_inc_in;
	this->slider_init = slider_init_in;

	
	d_slider = createSlider(this, sliderType);

#if QT_VERSION < 0x040000
    int alignment = Qt::AlignCenter;
#else
    QFlags<Qt::AlignmentFlag> alignment;
#endif
  /*  switch(d_slider->scalePosition())
    {
        case QwtSlider::NoScale:
            if ( d_slider->orientation() == Qt::Horizontal )
                alignment = Qt::AlignHCenter | Qt::AlignTop;
            else
                alignment = Qt::AlignVCenter | Qt::AlignLeft;
            break;
        case QwtSlider::LeftScale:
            alignment = Qt::AlignVCenter | Qt::AlignRight;
            break;
        case QwtSlider::RightScale:
            alignment = Qt::AlignVCenter | Qt::AlignLeft;
            break;
        case QwtSlider::TopScale:
            alignment = Qt::AlignHCenter | Qt::AlignBottom;
            break;
        case QwtSlider::BottomScale:
            alignment = Qt::AlignHCenter | Qt::AlignTop;
            break;
    }*/


	
		 
    d_label = new QLabel("0", this);
    d_label->setAlignment(alignment);
    d_label->setFixedWidth(d_label->fontMetrics().width("10000.9"));
    QRect pos_d_label = QRect(220, 35, 380, 30);
    d_label->setGeometry(pos_d_label);
    
    text_label = new QLabel("0", this);
    QRect pos_text_label = QRect(220,10,150,20);
    text_label->setGeometry(pos_text_label);
    text_label->setText(pre_text);

    double dummy = 20.0;
    
    connect(d_slider, SIGNAL(valueChanged(double)), this, SLOT(setNum(double)));
    
    QString text;
    text.setNum(d_slider->value() , 'f', 2);
    d_label->setText(text);




    QBoxLayout *layout;
    if ( d_slider->orientation() == Qt::Horizontal )
        layout = new QHBoxLayout(this);
    else
        layout = new QVBoxLayout(this);

    //layout->addWidget(d_slider);
    //layout->addWidget(d_label);
}

QwtSlider *Slider::createSlider(QWidget *parent, int sliderType) const
{
    QwtSlider *slider = NULL;
    
    switch(sliderType)
    {
        case 0:
        {
           slider = new QwtSlider(parent, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough);
           slider->setFont(QFont("Times", 10));
           slider->setHandleSize( 20, 6 );
           slider->setFixedWidth(200);
           slider->setRange(slider_min, slider_max, slider_inc, 0); // paging disabled
           slider->setValue(slider_init);
            break;
        }
        case 1:
        {
            slider = new QwtSlider(parent, Qt::Horizontal, QwtSlider::NoScale, QwtSlider::Trough);
            slider->setRange(slider_min, slider_max, slider_inc, 5);
            slider->setValue(slider_init);
            break;
        }
        case 2:
        {
            slider = new QwtSlider(parent, 
                Qt::Horizontal, QwtSlider::BottomScale, QwtSlider::Trough);
            //slider->setThumbWidth(25);
            //slider->setThumbLength(12);
            slider->setRange(slider_min, slider_max, slider_inc, 10);
            slider->setValue(slider_init);
            break;
        }
        case 3:
        {
            slider = new QwtSlider(parent, 
                Qt::Vertical, QwtSlider::LeftScale, QwtSlider::Trough);
            slider->setRange(slider_min, slider_max, slider_inc, 5);
            slider->setScaleMaxMinor(5);
            slider->setValue(slider_init);
            break;
        }
        case 4:
        {
            slider = new QwtSlider(parent, 
                Qt::Vertical, QwtSlider::NoScale, QwtSlider::Trough);
            slider->setRange(slider_min, slider_max, slider_inc, 10);
            slider->setValue(slider_init);
            break;
        }
        case 5:
        {
            slider = new QwtSlider(parent, 
                Qt::Vertical, QwtSlider::RightScale, QwtSlider::Trough);
            slider->setScaleEngine(new QwtLog10ScaleEngine);
            //slider->setThumbWidth(20);
            slider->setBorderWidth(1);
            slider->setRange(slider_min, slider_max, slider_inc, 1);
            slider->setScale(1.0, 1.0e4);
            slider->setScaleMaxMinor(10);
            slider->setValue(slider_init);
            break;
        }
    }

    QRect pos_slider = QRect(0, 10, 380, 40);
    slider->setGeometry(pos_slider);

    return slider;
}

void Slider::setNum(double v)
{
    if ( d_slider->scaleMap().transformation()->type() ==
        QwtScaleTransformation::Log10 )
    {
        v = pow(10.0, v);
    }
    
    QString text;
    text.setNum(d_slider->value() , 'f', 2);
    d_label->setText(text);
}

