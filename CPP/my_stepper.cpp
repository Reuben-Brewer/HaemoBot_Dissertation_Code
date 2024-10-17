#include "my_stepper.h"
#include "my_stepper_programmer.h"

my_stepper::my_stepper(QWidget *parent, QString stepper_name_in, int stepper_ID_in, float pos_min_in, float pos_max_in, float pos_inc_in, float pos_init_in, float vel_min_in, float vel_max_in, float vel_inc_in, float vel_init_in, float home_speed_min_in, float home_speed_max_in, float home_speed_inc_in, float home_speed_init_in, float preset_angle_1_in, float preset_angle_2_in, my_stepper_programmer *stepper_programmer_in):
    QWidget(parent)
{
   stepper_programmer = stepper_programmer_in;
   stepper_ID = stepper_ID_in;
   stepper_name = stepper_name_in.toStdString();
   preset_angle_1 = preset_angle_1_in;
   preset_angle_2 = preset_angle_2_in;
   pos_min = pos_min_in;
   pos_max = pos_max_in;
   goal_stepper_pos_double = pos_init_in;
   goal_stepper_vel_double = vel_init_in;

   /////////////////////////background_box must be the first GUI object so that it doesn't cover over other objects!!!
	background_box = new QLabel("",this);
	background_box->setGeometry(0,0,400,490);
	background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(rgb(200,255,200))}");

	stepper_ID_spinbox = new print_labeled_spinbox(this, "Stepper ID: ", 1.0, 0.0, 0.0, 255.0, stepper_ID);
   QRect pos_stepper_ID_spinbox = QRect(20, 5, 150, 30);
   stepper_ID_spinbox->setGeometry(pos_stepper_ID_spinbox);
   connect(stepper_ID_spinbox->my_spin_box, SIGNAL(valueChanged(double)), this, SLOT(change_stepper_ID(double)));

   enable_button = new QRadioButton("Enable", this);
   QRect pos_enable_button = QRect(165,5,120,20);
   enable_button->setGeometry(pos_enable_button);
   enable_button->setAutoExclusive(0);
   enable_button->setChecked(1);
   connect(enable_button, SIGNAL(toggled(bool)), this, SLOT(change_enabling(bool)));

   homeButton = new QPushButton("Not Homed", this);
   QRect pos_homeButton = QRect(250,5,120,20);
   homeButton->setGeometry(pos_homeButton);
   connect(homeButton, SIGNAL(clicked()), this, SLOT(home()));

   goToButton = new QPushButton("Go To Preset", this);
   QRect pos_goToButton = QRect(250,35,120,20);
   goToButton->setGeometry(pos_goToButton);
   connect(goToButton, SIGNAL(clicked()), this, SLOT(goToPreset()));

   step_mode_button = new QPushButton("Using Full Step", this);
   QRect pos_step_mode_button = QRect(250,65,120,20);
   step_mode_button->setGeometry(pos_step_mode_button);
   connect(step_mode_button, SIGNAL(clicked()), this, SLOT(change_step_mode()));

   stepper_mutex = new QMutex;

   int plot_height = 175;
   int plot_width = 370;
   plot_data_vec.push_back(&actual_stepper_pos_double);
   plot_data_vec.push_back(&goal_stepper_pos_double);
   plot = new DataPlot(plot_data_vec, 1, pos_min, pos_max, plot_height, QString(stepper_name.c_str()), "ticks", 0, this, stepper_mutex);
   QRect pos_plot = QRect(5, 95, plot_width, plot_height);
   plot->setGeometry(pos_plot);

   slider_box_pos = new slider_with_box(this, "Pos", 0, pos_min_in, pos_max_in, pos_inc_in, pos_init_in);
   slider_box_pos->setGeometry(QRect(5, 280, 380, 60));
   connect(slider_box_pos, SIGNAL(valueChanged(double)), this, SLOT(pos_or_vel_slider_changed()));

   slider_box_vel = new slider_with_box(this, "Vel", 0, vel_min_in, vel_max_in, vel_inc_in, vel_init_in);
   slider_box_vel->setGeometry(QRect(5, 345, 380, 60));
   connect(slider_box_vel, SIGNAL(valueChanged(double)), this, SLOT(pos_or_vel_slider_changed()));

   slider_box_home_speed = new slider_with_box(this, "Home Speed", 0, home_speed_min_in, home_speed_max_in, home_speed_inc_in, home_speed_init_in);
   slider_box_home_speed->setGeometry(QRect(5, 410, 380, 60));
   connect(slider_box_home_speed, SIGNAL(valueChanged(double)), this, SLOT(pos_or_vel_slider_changed()));

   //enable();

   goal_stepper_pos_double = pos_init_in;
   goal_stepper_vel_double = vel_init_in;
   home_speed = home_speed_init_in;
   toBeHomed = 1; //default to homing when we first get created
   homed_state = -1;
   step_mode = 0; //defaults to fullstep
   isDraggedFlag = false;
}

void my_stepper::enable()
{
        enabled_flag = true;
        slider_box_pos->enable();
        slider_box_vel->enable();
        slider_box_home_speed->enable();
        goToButton->setEnabled(1);
}

void my_stepper::disable()
{
        enabled_flag = false;
        slider_box_pos->disable();
        slider_box_vel->disable();
        slider_box_home_speed->disable();
        goToButton->setEnabled(0);
}

void my_stepper::change_enabling(bool checked)
{
        if(checked == true)
                enable();
        else
                disable();
}

void my_stepper::pos_or_vel_slider_changed()
{
    goal_stepper_pos_double = slider_box_pos->value;
    goal_stepper_vel_double = slider_box_vel->value;
    home_speed = slider_box_home_speed->value;
    isDraggedFlag = true;
}

void my_stepper::goToPreset(void)
{
        static bool last_end_position = 0;

        if(last_end_position)
        {
                last_end_position = 0;
                goal_stepper_pos_double = preset_angle_1;
        }
        else
        {
                last_end_position = 1;
                goal_stepper_pos_double = preset_angle_2;
        }
        //forced_update();
}

void my_stepper::move_stepper(double goal_pos, double goal_vel)
{
        //stepper_programmer->move_stepper(stepper_ID, goal_pos, goal_vel);
        isDraggedFlag = false;
}

void my_stepper::timer_update(QTimerEvent *event)
{
         if(homed_state == -1)
        {
            homeButton->setText("Not Homed");
            //homeButton->setEnabled(1);
        }
        else if(homed_state == 0)
        {
            homeButton->setText("Homing");
            //homeButton->setEnabled(0);
        }
        else if(homed_state == 1)
        {
            homeButton->setText("Homed");
            //homeButton->setEnabled(1);
        }

        plot->update_data_plot(event); //mutex gets locked within this function
        if(isDraggedFlag == false)
        {
                slider_box_pos->setVal(goal_stepper_pos_double);
                slider_box_vel->setVal(goal_stepper_vel_double);
                slider_box_home_speed->setVal(home_speed);
        }
}

void my_stepper::forced_update(void)
{
        if(isEnabled() == true)
        {
                //move_stepper(goal_stepper_pos_double, goal_stepper_vel_double);


                //double temp  = stepper_programmer->readPosVel(stepper_ID);
                /*if(temp != -1)
                {
                        stepper_mutex->lock();
                        actual_stepper_pos_double = temp;
                        stepper_mutex->unlock();
                }*/

        }

}

bool my_stepper::isEnabled(void)
{
        return enabled_flag;
}

void my_stepper::change_stepper_ID(double new_stepper_ID)
{
        stepper_ID = new_stepper_ID;
}

void my_stepper::change_step_mode()
{
    if(step_mode == 0)
    {
        step_mode = 1;
        step_mode_button->setText("Using Half Step");
        slider_box_vel->setRange(1,100,1);
    }
    else if(step_mode == 1)
    {
        step_mode = 0;
        step_mode_button->setText("Using Full Step");
        slider_box_vel->setRange(2,100,1);
        if(goal_stepper_vel_double < 2.0)
        {
            goal_stepper_vel_double = 2.0;
            slider_box_vel->setVal(goal_stepper_vel_double);
        }
    }
}

void my_stepper::home()
{
    //if(toBeHomed == 0) //(just can't issue a HOME command when already homing)
    //{
        toBeHomed = 1;
        stepper_programmer->message_to_be_sent = 2;
        //homeButton->setEnabled(0);
    //}
}
