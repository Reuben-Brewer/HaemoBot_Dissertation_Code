#include "dynamixel_servo.h"
#include "dynamixel_servo_programmer.h"

dynamixel_servo::dynamixel_servo(QWidget *parent, QString servo_name_in, int servo_ID_in, float pos_min_in, float pos_max_in, float pos_inc_in, float pos_init_in, float vel_min_in, float vel_max_in, float vel_inc_in, float vel_init_in, float preset_angle_1_in, float preset_angle_2_in, dynamixel_servo_programmer *servo_programmer_in):
    QWidget(parent)
{
   servo_programmer = servo_programmer_in;
   servo_ID = servo_ID_in;
   servo_name = servo_name_in.toStdString();
   preset_angle_1 = preset_angle_1_in;
   preset_angle_2 = preset_angle_2_in;
   pos_min = pos_min_in;
   pos_max = pos_max_in;
   goal_servo_pos_double = pos_init_in;
   goal_servo_vel_double = vel_init_in;

	/////////////////////////background_box must be the first GUI object so that it doesn't cover over other objects!!!
	background_box = new QLabel("",this);
	background_box->setGeometry(0,0,385,385);
	background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(rgb(200,255,200))}");

   servo_ID_spinbox = new print_labeled_spinbox(this, "Servo ID", 1.0, 0.0, 0.0, 255.0, servo_ID);
   QRect pos_servo_ID_spinbox = QRect(20, 5, 150, 30);
   servo_ID_spinbox->setGeometry(pos_servo_ID_spinbox);
   connect(servo_ID_spinbox->my_spin_box, SIGNAL(valueChanged(double)), this, SLOT(change_servo_ID(double)));

   enable_button = new QRadioButton("Enable", this);
   QRect pos_enable_button = QRect(170,5,120,20);
   enable_button->setGeometry(pos_enable_button);
   enable_button->setAutoExclusive(0);
   enabled_flag = true;
   enable_button->setChecked(enabled_flag);
   connect(enable_button, SIGNAL(toggled(bool)), this, SLOT(change_enabling(bool)));

   goToButton = new QPushButton("Go To Preset", this);
   QRect pos_goToButton = QRect(235,5,120,20);
   goToButton->setGeometry(pos_goToButton);
   connect(goToButton, SIGNAL(clicked()), this, SLOT(goToPresetToggleButtonFunction()));
   last_end_position = 0;

   servo_mutex = new QMutex;

   int plot_height = 175;
   int plot_width = 370;
   plot_data_vec.push_back(&goal_servo_pos_double);
   plot_data_vec.push_back(&actual_servo_pos_double);
   plot = new DataPlot(plot_data_vec, 0, pos_min, pos_max, plot_height, QString(servo_name.c_str()), "ticks", 0, this, servo_mutex);
   QRect pos_plot = QRect(0, 45, plot_width, plot_height);
   plot->setGeometry(pos_plot);

   slider_box_pos = new slider_with_box(this, "Pos", 0, pos_min_in, pos_max_in, pos_inc_in, pos_init_in);
   QRect pos_slider_box_pos = QRect(5, 230, 380, 60);
   slider_box_pos->setGeometry(pos_slider_box_pos);
   connect(slider_box_pos, SIGNAL(valueChanged(double)), this, SLOT(pos_or_vel_slider_changed()));

   slider_box_vel = new slider_with_box(this, "Vel", 0, vel_min_in, vel_max_in, vel_inc_in, vel_init_in);
   QRect pos_slider_box_vel = QRect(5, 295, 380, 60);
   slider_box_vel->setGeometry(pos_slider_box_vel);
   connect(slider_box_vel, SIGNAL(valueChanged(double)), this, SLOT(pos_or_vel_slider_changed()));

   enable();

   isDraggedFlag = false;
}

void dynamixel_servo::enable()
{
	enabled_flag = true;
	slider_box_pos->enable();
	slider_box_vel->enable();
	goToButton->setEnabled(1);
        //servo_programmer->toggleLED(servo_ID, true);
}

void dynamixel_servo::disable()
{
	enabled_flag = false;
	slider_box_pos->disable();
	slider_box_vel->disable();
	goToButton->setEnabled(0);
	servo_programmer->toggleLED(servo_ID, false);
}

void dynamixel_servo::change_enabling(bool checked)
{
	if(checked == true)
		enable();
	else
		disable();
}

void dynamixel_servo::pos_or_vel_slider_changed()
{
	goal_servo_pos_double = slider_box_pos->value;
	goal_servo_vel_double = slider_box_vel->value;
	isDraggedFlag = true;
}

void dynamixel_servo::goToPresetToggleButtonFunction(void)
{
	if(last_end_position == 1)
	{
		goToPresetPosition(0);
	}
	else
	{
		goToPresetPosition(1);
	}
}

void dynamixel_servo::goToPresetPosition(bool val)
{
	if(val == 0)
	{
		last_end_position = 0;
		goal_servo_pos_double = preset_angle_1;
	}
	else if(val == 1)
	{
		last_end_position = 1;
		goal_servo_pos_double = preset_angle_2;
	}
}

void dynamixel_servo::move_servo(double goal_pos, double goal_vel)
{
	servo_programmer->move_servo(servo_ID, goal_pos, goal_vel);



	isDraggedFlag = false;

}

void dynamixel_servo::timer_update(QTimerEvent *event)
{
	plot->update_data_plot(event); //mutex gets locked within this function
        if(isDraggedFlag == false)
        {
                slider_box_pos->setVal(goal_servo_pos_double);
                slider_box_vel->setVal(goal_servo_vel_double);
        }
}

void dynamixel_servo::forced_update(void)
{
	if(isEnabled() == true)
	{
                move_servo(goal_servo_pos_double, goal_servo_vel_double);
				Sleep(5);
                double temp  = servo_programmer->readPosVel(servo_ID);
                if(temp != -1)
                {
                        servo_mutex->lock();
                        actual_servo_pos_double = temp;
                        servo_mutex->unlock();
                }
		
	}

}

bool dynamixel_servo::isEnabled(void)
{
	return enabled_flag;
}

void dynamixel_servo::change_servo_ID(double new_servo_ID)
{
	servo_ID = new_servo_ID;
}
