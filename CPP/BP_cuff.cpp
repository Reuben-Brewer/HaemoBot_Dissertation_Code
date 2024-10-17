#include "BP_cuff.h"
#include "my_stepper_programmer.h"

BP_cuff::BP_cuff(QWidget *parent, float pressure_min_in, float pressure_max_in, float pressure_inc_in, float pressure_init_in, my_stepper_programmer *stepper_programmer_in):
    QWidget(parent)
{
   stepper_programmer = stepper_programmer_in;
   goal_pressure_double = pressure_init_in;

   /////////////////////////background_box must be the first GUI object so that it doesn't cover over other objects!!!
	background_box = new QLabel("",this);
	background_box->setGeometry(0,0,380,280);
	background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(rgb(200,255,200))}");

   enable_button = new QRadioButton("Enable", this);
   QRect pos_enable_button = QRect(5,5,120,20);
   enable_button->setGeometry(pos_enable_button);
   enable_button->setAutoExclusive(0);
   enable_button->setChecked(1);
   connect(enable_button, SIGNAL(toggled(bool)), this, SLOT(change_enabling(bool)));

   stepper_mutex = new QMutex;

   int plot_height = 175;
   int plot_width = 370;
   std::vector<double *> plot_data_vec;
   plot_data_vec.push_back(&actual_pressure_double);
   plot_data_vec.push_back(&goal_pressure_double);
   plot = new DataPlot(plot_data_vec, 1, pressure_min_in, pressure_max_in, plot_height, QString("BP"), "ticks", 0, this, stepper_mutex); //NULL used to be stepper_mutex
   plot->setGeometry(QRect(5, 25, plot_width, plot_height));

   slider_box = new slider_with_box(this, "BP", 0, pressure_min_in, pressure_max_in, pressure_inc_in, pressure_init_in);
   slider_box->setGeometry(QRect(5, 205, 380, 60));
   connect(slider_box, SIGNAL(valueChanged(double)), this, SLOT(slider_changed()));

   //enable();

   isDraggedFlag = false;
}

void BP_cuff::enable()
{
        enabled_flag = true;
        slider_box->enable();
}

void BP_cuff::disable()
{
        enabled_flag = false;
        slider_box->disable();
}

void BP_cuff::change_enabling(bool checked)
{
        if(checked == true)
                enable();
        else
                disable();
}

void BP_cuff::slider_changed()
{
        goal_pressure_double = slider_box->value;
        isDraggedFlag = true;
}


void BP_cuff::timer_update(QTimerEvent *event)
{ 
        plot->update_data_plot(event); //mutex gets locked within this function
        if(isDraggedFlag == false)
        {
			slider_box->setVal(goal_pressure_double);
        }
}

void BP_cuff::forced_update(void)
{
        if(isEnabled() == true)
        {

        }

}

bool BP_cuff::isEnabled(void)
{
        return enabled_flag;
}




