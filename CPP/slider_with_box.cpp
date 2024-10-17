#include "slider_with_box.h"

slider_with_box::slider_with_box(QWidget *parent, QString pre_text_in, int sliderType_in, float slider_min_in, float slider_max_in, float slider_inc_in, float slider_init_in):
    QWidget(parent)
{
    slider_init = slider_init_in;

    spinbox = new print_labeled_spinbox(this, "", slider_inc_in, 3, slider_min_in, slider_max_in, slider_init_in);
    QRect pos_spinbox = QRect(0,30,60,30);
    spinbox->setGeometry(pos_spinbox);
    connect(spinbox->my_spin_box, SIGNAL(valueChanged(double)), this, SLOT(change_slider()));

    slider = new Slider(this, pre_text_in, sliderType_in, slider_min_in, slider_max_in, slider_inc_in, slider_init_in);
    QRect pos_slider = QRect(70, 0, 380, 100);
    slider->setGeometry(pos_slider);
    connect(slider->d_slider, SIGNAL(sliderMoved(double)), this, SLOT(change_spinbox()));

	isPressed = false;
    value = slider_init_in;
}

void slider_with_box::change_spinbox()
{
    float slider_val = slider->d_slider->value();
    spinbox->my_spin_box->setValue(slider_val);
	value = slider_val;
        Q_EMIT valueChanged(value);
}

void slider_with_box::change_slider()
{
    float spinbox_val = spinbox->my_spin_box->value();
    slider->d_slider->setValue(spinbox_val);
	value = spinbox_val;
        Q_EMIT valueChanged(value);
}

void slider_with_box::enable()
{
    slider->d_slider->setEnabled(1);
    spinbox->my_spin_box->setEnabled(1);
}

void slider_with_box::disable()
{
    slider->d_slider->setEnabled(0);
    spinbox->my_spin_box->setEnabled(0);
}

void slider_with_box::zero()
{
    slider->d_slider->setValue(slider_init);
    spinbox->my_spin_box->setValue(slider_init);
}

float slider_with_box::getSpinBoxVal()
{
    return spinbox->my_spin_box->value();
}

float slider_with_box::getSliderVal()
{
    return slider->d_slider->value();
}

void slider_with_box::setSliderVal(double val)
{
	slider->d_slider->setValue(val);
}

void slider_with_box::setSpinBoxVal(double val)
{
	spinbox->my_spin_box->setValue(val);
}

void slider_with_box::setVal(double val)
{
	setSliderVal(val);
        setSpinBoxVal(val);
	value = val;
}

void slider_with_box::setRange(double min, double max, double inc)
{
    slider->d_slider->setRange(min, max, inc, 1);
    spinbox->my_spin_box->setRange(min, max);
}

