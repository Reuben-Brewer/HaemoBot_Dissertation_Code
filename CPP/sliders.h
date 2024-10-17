#ifndef _SLIDERS_H
#define _SLIDERS_H 1

#include <qwidget.h>
#include <qwt_slider.h>

class QLabel;
class QLayout;

class Slider: public QWidget
{
    Q_OBJECT
public:
    Slider(QWidget *parent, QString pre_text_in = "", int sliderType_in = 0, float slider_min_in = 0.0, float slider_max_in = 0.0, float slider_inc_in = 0.0, float slider_init_in = 0.0);
    QwtSlider *createSlider(QWidget *, int sliderType) const;

    QwtSlider *d_slider;
    QLabel *d_label, *text_label;
    QString pre_text;
    int sliderType;
    float slider_min, slider_max, slider_inc, slider_init;
    

private Q_SLOTS:
    void setNum(double v);

private:

};



#endif
