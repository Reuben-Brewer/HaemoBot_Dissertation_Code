#ifndef BP_CUFF_H
#define BP_CUFF_H

#include "slider_with_box.h"
#include <string>
#include <vector>
#include <QRadioButton>
#include <QPushButton>
#include "rlserial.h"
#include "data_plot.h"
#include <QMutex>
#include "my_stepper_programmer.h"
#include "print_labeled_spinbox.h"
#include <QLabel>

class BP_cuff: public QWidget
{
    Q_OBJECT
public:
    BP_cuff(QWidget *parent, float pressure_min_in = 0.0, float pressure_max_in = 0.0, float pressure_inc_in = 0.0, float pressure_init_in = 0.0, my_stepper_programmer *stepper_programmer_in = NULL);
    void timer_update(QTimerEvent *e);
    void enable(void);
    void disable(void);
    void forced_update(void);
    bool isEnabled(void);

    slider_with_box *slider_box;
    DataPlot *plot;
    QRadioButton *enable_button;

    double goal_pressure_double, actual_pressure_double;
    my_stepper_programmer *stepper_programmer;
    bool isDraggedFlag, enabled_flag;
    QMutex *stepper_mutex;
	QLabel *background_box;


private Q_SLOTS:
        void change_enabling(bool checked);
        void slider_changed(void);
};



#endif
