#include "my_DC_motor.h"

my_DC_motor::my_DC_motor(QWidget *parent, QString motor_name_in, int motor_ID_in, float CPR_in, float kp_min_in, float kp_max_in, float kp_inc_in, float kp_init_in, float kv_min_in, float kv_max_in, float kv_inc_in, float kv_init_in, float ki_min_in, float ki_max_in, float ki_inc_in, float ki_init_in, float pos_min_in, float pos_max_in, float pos_inc_in, float pos_init_in, float vel_min_in, float vel_max_in, float vel_inc_in, float vel_init_in, float accel_min_in, float accel_max_in, float accel_inc_in, float accel_init_in):
    QWidget(parent)
{
   motor_ID = motor_ID_in;
   motor_name = motor_name_in.toStdString();
   CPR = CPR_in;

   enable_button = new QRadioButton("Enable", this);
   QRect pos_enable_button = QRect(0,0,120,20);
   enable_button->setGeometry(pos_enable_button);
   enable_button->setAutoExclusive(0);
   enable_button->setChecked(1);
   connect(enable_button, SIGNAL(toggled(bool)), this, SLOT(change_enabling(bool)));

   vel_control_button = new QRadioButton("Vel Control", this);
   QRect pos_vel_control_button = QRect(165,0,120,20);
   vel_control_button->setGeometry(pos_vel_control_button);
   vel_control_button->setAutoExclusive(0);
   vel_control_button->setChecked(0);
   connect(vel_control_button, SIGNAL(toggled(bool)), this, SLOT(change_vel_control_mode(bool)));
   vel_control_mode = false;

   homeButton = new QPushButton("Not Homed", this);
   QRect pos_homeButton = QRect(250,0,120,20);
   homeButton->setGeometry(pos_homeButton);
   connect(homeButton, SIGNAL(clicked()), this, SLOT(home()));

   motor_mutex = new QMutex;

   int plot_height = 175;
   int plot_width = 370;
   plot_data_vec.push_back(&actual_pos);
   plot_data_vec.push_back(&goal_pos);
   plot = new DataPlot(plot_data_vec, 1, vel_min_in, vel_max_in, plot_height, QString(motor_name.c_str()), "ticks", 0, this, motor_mutex);
   QRect pos_plot = QRect(0, 30, plot_width, plot_height);
   plot->setGeometry(pos_plot);

   int slider_x_start = 0;
   int slider_y_start = 215;
   int slider_x_inc = 10;
   int slider_y_inc = 5;
   int slider_width = 370;
   int slider_height = 55;

   slider_box_pos = new slider_with_box(this, "Pos", 0, pos_min_in, pos_max_in, pos_inc_in, pos_init_in);
   slider_box_pos->setGeometry(QRect(slider_x_start + 0*(slider_x_inc + slider_width), slider_y_start + 0*(slider_y_inc + slider_height), slider_width, slider_height));
   connect(slider_box_pos, SIGNAL(valueChanged(double)), this, SLOT(slider_changed()));

   slider_box_vel = new slider_with_box(this, "Vel", 0, vel_min_in, vel_max_in, vel_inc_in, vel_init_in);
   slider_box_vel->setGeometry(QRect(slider_x_start + 0*(slider_x_inc + slider_width), slider_y_start + 1*(slider_y_inc + slider_height), slider_width, slider_height));
   connect(slider_box_vel, SIGNAL(valueChanged(double)), this, SLOT(slider_changed()));


   slider_box_accel = new slider_with_box(this, "Accel", 0, accel_min_in, accel_max_in, accel_inc_in, accel_init_in);
   slider_box_accel->setGeometry(QRect(slider_x_start + 0*(slider_x_inc + slider_width), slider_y_start + 2*(slider_y_inc + slider_height), slider_width, slider_height));
   connect(slider_box_accel, SIGNAL(valueChanged(double)), this, SLOT(slider_changed()));

   slider_box_kp = new slider_with_box(this, "Kp", 0, kp_min_in, kp_max_in, kp_inc_in, kp_init_in);
   slider_box_kp->setGeometry(QRect(slider_x_start + 0*(slider_x_inc + slider_width), slider_y_start + 3*(slider_y_inc + slider_height), slider_width, slider_height));
   connect(slider_box_kp, SIGNAL(valueChanged(double)), this, SLOT(slider_changed()));

   slider_box_kv = new slider_with_box(this, "Kv", 0, kv_min_in, kv_max_in, kv_inc_in, kv_init_in);
   slider_box_kv->setGeometry(QRect(slider_x_start + 0*(slider_x_inc + slider_width), slider_y_start + 4*(slider_y_inc + slider_height), slider_width, slider_height));
   connect(slider_box_kv, SIGNAL(valueChanged(double)), this, SLOT(slider_changed()));

   slider_box_ki = new slider_with_box(this, "Ki", 0, ki_min_in, ki_max_in, ki_inc_in, ki_init_in);
   slider_box_ki->setGeometry(QRect(slider_x_start + 0*(slider_x_inc + slider_width), slider_y_start + 5*(slider_y_inc + slider_height), slider_width, slider_height));
   connect(slider_box_ki, SIGNAL(valueChanged(double)), this, SLOT(slider_changed()));

   open(360.0);

   my_DC_phidget_motor->kp = kp_init_in;
   my_DC_phidget_motor->kv = kv_init_in;
   my_DC_phidget_motor->ki = ki_init_in;

   my_DC_phidget_motor->goal_position_degrees = pos_init_in;
   my_DC_phidget_motor->goal_velocity_degrees = vel_init_in;
   my_DC_phidget_motor->changeAcceleration(accel_init_in);

   toBeHomed = 1; //default to homing when we first get created
   homed_state = -1;
   isDraggedFlag = false;
   enable();
   slider_box_vel->disable();
}

void my_DC_motor::enable()
{
        enabled_flag = true;
        /*slider_box_kp->enable();
        slider_box_kv->enable();
        slider_box_ki->enable();
        slider_box_pos->enable();
        slider_box_vel->enable();
        slider_box_accel->enable();*/
}

void my_DC_motor::open(double CPR)
{
    my_DC_phidget_motor = new phidget_DC_motor(CPR);
}

void my_DC_motor::close()
{
    my_DC_phidget_motor->close();
}

void my_DC_motor::disable()
{
        enabled_flag = false;
        /*slider_box_kp->disable();
        slider_box_kv->disable();
        slider_box_ki->disable();
        slider_box_pos->disable();
        slider_box_vel->disable();
        slider_box_accel->disable();*/
        my_DC_phidget_motor->changeVelocity(0);
}

void my_DC_motor::change_enabling(bool checked)
{
        if(checked == true)
                enable();
        else
                disable();
}

void my_DC_motor::change_vel_control_mode(bool checked)
{
        if(checked == true)
        {
                vel_control_mode = true;
                slider_box_vel->enable();
                slider_box_pos->disable();
        }
        else
        {
                vel_control_mode = false;
                slider_box_vel->disable();
                slider_box_pos->enable();
                my_DC_phidget_motor->changeVelocity(0);
                goal_pos = actual_pos;
                slider_box_pos->setVal(actual_pos);
                my_DC_phidget_motor->goal_position_degrees = goal_pos;
        }
}

void my_DC_motor::slider_changed()
{
    printf("ffff \n");
    goal_pos = slider_box_pos->value;
    goal_vel = slider_box_vel->value;
    goal_accel = slider_box_accel->value; 

    kp = slider_box_kp->value;
    kv = slider_box_kv->value;
    ki = slider_box_ki->value;

    my_DC_phidget_motor->kp = kp;
    my_DC_phidget_motor->kv = kv;
    my_DC_phidget_motor->ki = ki;

    my_DC_phidget_motor->goal_position_degrees = goal_pos;
    my_DC_phidget_motor->changeAcceleration(goal_accel);

    isDraggedFlag = true;
}


void my_DC_motor::move_motor(void)
{
    if(enabled_flag == true)
    {
        if(vel_control_mode == true)
        {
            my_DC_phidget_motor->readEncoder();
            my_DC_phidget_motor->readVelocity();
            my_DC_phidget_motor->changeVelocity(goal_vel);
        }
        else
        {
            my_DC_phidget_motor->updatePID();
        }
    }

        isDraggedFlag = false;
}

void my_DC_motor::timer_update(QTimerEvent *event)
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

        updatePosition();
        plot->update_data_plot(event); //mutex gets locked within this function
        if(isDraggedFlag == false)
        {
                slider_box_pos->setVal(goal_pos);
                slider_box_vel->setVal(goal_vel);
                slider_box_accel->setVal(goal_accel);
        }
}


bool my_DC_motor::isEnabled(void)
{
        return enabled_flag;
}



void my_DC_motor::home()
{
    //if(toBeHomed == 0) //(just can't issue a HOME command when already homing)
    //{
        my_DC_phidget_motor->setEncoder(0);
        toBeHomed = 1;
        //homeButton->setEnabled(0);
    //}
}

void my_DC_motor::updatePosition(void)
{
    actual_pos = my_DC_phidget_motor->actual_position_degrees; //in degrees
    actual_vel = my_DC_phidget_motor->actual_velocity_degrees; //in degrees/s;
}
