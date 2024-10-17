#include "my_servo_motor.h"

my_servo::my_servo(QWidget *parent, vector<double> servo_start_angle_in):
    QWidget(parent)
{



   motor_mutex = new QMutex;

   num_servos = servo_start_angle_in.size();

   my_servo_motor = new phidget_servo_motor(num_servos);



   int slider_x_start = 0;
   int slider_y_start = 205;
   int slider_x_inc = 10;
   int slider_y_inc = 10;
   int slider_width = 370;
   int slider_height = 60;

   int plot_height = 175;
   int plot_width = 370;

   for(int index = 0; index < num_servos; index++)
   {
        enabled_flag.push_back(true);
        actual_pos.push_back(0);
        actual_vel.push_back(0);
        actual_accel.push_back(0);
        goal_pos.push_back(servo_start_angle_in[0]);
        goal_vel.push_back(my_servo_motor->max_vel);
        goal_accel.push_back(my_servo_motor->min_accel);

        slider_box_pos.push_back(new slider_with_box(this, "Pos", 0, my_servo_motor->min_pos, my_servo_motor->max_pos, 1, servo_start_angle_in[0]));
        slider_box_pos[index]->setGeometry(QRect(slider_x_start + index*(slider_x_inc + slider_width), slider_y_start + 0*(slider_y_inc + slider_height), slider_width, slider_height));
        connect(slider_box_pos[index], SIGNAL(valueChanged(double)), this, SLOT(pos_slider_changed()));

        slider_box_vel.push_back(new slider_with_box(this, "Vel", 0, my_servo_motor->min_vel, my_servo_motor->max_vel, 1, my_servo_motor->max_vel));
        slider_box_vel[index]->setGeometry(QRect(slider_x_start + index*(slider_x_inc + slider_width), slider_y_start + 1*(slider_y_inc + slider_height), slider_width, slider_height));
        connect(slider_box_vel[index], SIGNAL(valueChanged(double)), this, SLOT(vel_or_accel_slider_changed()));

        slider_box_accel.push_back(new slider_with_box(this, "Accel", 0, my_servo_motor->min_accel, my_servo_motor->max_accel, 1, my_servo_motor->min_accel));
        slider_box_accel[index]->setGeometry(QRect(slider_x_start + index*(slider_x_inc + slider_width), slider_y_start + 2*(slider_y_inc + slider_height), slider_width, slider_height));
        connect(slider_box_accel[index], SIGNAL(valueChanged(double)), this, SLOT(vel_or_accel_slider_changed()));


        plot.push_back(new DataPlot(vector<double *>(2, NULL), 0, my_servo_motor->min_pos, my_servo_motor->max_pos, plot_height, QString::number(index), "ticks", 0, this, motor_mutex));
        plot[index]->setGeometry(QRect(slider_x_start + index*(slider_x_inc + slider_width), 0, plot_width, plot_height));


        enable_button.push_back(new QRadioButton("Enable", this));
        QRect pos_enable_button = QRect(240 + slider_x_start + index*(slider_x_inc + slider_width),0,120,20);
        enable_button[index]->setGeometry(pos_enable_button);
        enable_button[index]->setAutoExclusive(0);
        enable_button[index]->setChecked(1);
        connect(enable_button[index], SIGNAL(toggled(bool)), this, SLOT(change_enabling(bool)));

        enable(index);
        move_motor(index, servo_start_angle_in[0]);
   }

   for(int i = 0; i < num_servos; i++)
   {
        plot[i]->data_vector[0] = &goal_pos[i];
        plot[i]->data_vector[1] = &actual_pos[i];
   }







   isDraggedFlag = false;

}

void my_servo::enable(int num)
{
        enabled_flag[num] = true;
        slider_box_pos[num]->enable();
        slider_box_vel[num]->enable();
        slider_box_accel[num]->enable();
        my_servo_motor->setEnabled(num, true);
}

void my_servo::open()
{
   // my_servo_motor = new phidget_servo_motor(motor_ID, servo);
}

void my_servo::close()
{
    my_servo_motor->close();
}

void my_servo::disable(int num)
{
        enabled_flag[num] = false;
        slider_box_pos[num]->disable();
        slider_box_vel[num]->disable();
        slider_box_accel[num]->disable();
        my_servo_motor->setEnabled(num, false);
}

void my_servo::change_enabling(bool checked)
{
    for(int i = 0; i < num_servos; i++)
    {
        enabled_flag[i] = enable_button[i]->isChecked();
        if(enabled_flag[i] == true)
            enable(i);
        else
            disable(i);
    }
}

void my_servo::pos_slider_changed()
{
    for(int i = 0; i < num_servos; i++)
    {
        goal_pos[i] = slider_box_pos[i]->value;
    }
    isDraggedFlag = true;
}

void my_servo::vel_or_accel_slider_changed(void)
{
    for(int i = 0; i < num_servos; i++)
    {
        goal_vel[i] = slider_box_vel[i]->value;
        goal_accel[i] = slider_box_accel[i]->value;
        my_servo_motor->changeVelocityLimit(i, goal_vel[i]);
        my_servo_motor->changeAccelerationLimit(i, goal_accel[i]);
    }
    isDraggedFlag = true;
}

void my_servo::move_motor(int num, double pos)
{
        my_servo_motor->changePosition(num, pos);
        isDraggedFlag = false;
}

void my_servo::timer_update(QTimerEvent *event)
{
    for(int i = 0; i < num_servos; i++)
    {
        plot[i]->update_data_plot(event); //mutex gets locked within this function
        if(isDraggedFlag == false)
        {
                slider_box_pos[i]->setVal(goal_pos[i]);
                slider_box_vel[i]->setVal(goal_vel[i]);
                slider_box_accel[i]->setVal(goal_accel[i]);
        }
    }
}

bool my_servo::isEnabled(int num)
{
    return enabled_flag[num];
}

void my_servo::updatePosition(int num)
{
    actual_pos[num] = my_servo_motor->readPosition(num);
    actual_vel[num] = my_servo_motor->readVelocity(num);
}
