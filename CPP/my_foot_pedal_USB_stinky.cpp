#include "my_foot_pedal_USB_stinky.h"

my_foot_pedal_USB_stinky::my_foot_pedal_USB_stinky(QWidget *parent, std::string label_text): QWidget(parent)
{
    /////////////////////////background_box must be the first GUI object so that it doesn't cover over other objects!!!
	background_box = new QLabel("",this);
	background_box->setGeometry(0,0,500,200);
	background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(rgb(200,255,200))}");

	box_title = new QLabel("Foot Pedal",this);
	box_title->setGeometry(10,10,150,30);
	box_title->setFont(QFont("Times", 12, QFont::Bold));

	keyEvent_mutex = new QMutex();

	switch_1_BOOL_val = 0;
	switch_2_BOOL_val = 0;
	switch_3_BOOL_val = 0;
	switch_4_BOOL_val = 0;

	int label_start_x = 5;
	int label_start_y = 45;
	int label_width = 75;
	int label_height = 25;
	int label_width_inc = 5;
	int label_height_inc = 5;

	switch_1_label = new QLabel("", this);
    switch_1_label->setGeometry(QRect(label_start_x + 0*(label_width + label_width_inc), label_start_y + 0*(label_height + label_height_inc), label_width, label_height));

	switch_2_label = new QLabel("", this);
    switch_2_label->setGeometry(QRect(label_start_x + 0*(label_width + label_width_inc), label_start_y + 1*(label_height + label_height_inc), label_width, label_height));

	switch_3_label = new QLabel("", this);
    switch_3_label->setGeometry(QRect(label_start_x + 0*(label_width + label_width_inc), label_start_y + 2*(label_height + label_height_inc), label_width, label_height));

	switch_4_label = new QLabel("", this);
    switch_4_label->setGeometry(QRect(label_start_x + 0*(label_width + label_width_inc), label_start_y + 3*(label_height + label_height_inc), label_width, label_height));

	this->setFocusPolicy(Qt::StrongFocus);
	//this->setFocus(); //This line is necessary for this widget to receive the KeyEvent.
	do_i_have_focus_flag = 0;
}


void my_foot_pedal_USB_stinky::mousePressEvent(QMouseEvent *incoming_mouse_event) 
{
    
}

void my_foot_pedal_USB_stinky::keyPressEvent(QKeyEvent *incoming_key_event) //Function must be named "keyPressEvent" to properly overide the normal even handler and operate correctly.
{
	
	incoming_key_event->accept();
	keyQString = incoming_key_event->text();
	
	keyEvent_mutex->lock();
		if(keyQString == "1")
		{
			switch_1_BOOL_val = !switch_1_BOOL_val;
			//std::cout << "Switch 1 detected in my_foot_pedal_USB_stinky: " << keyQString.toStdString() << std::endl;
		}
		else if(keyQString == QString::QString("2"))
		{
			switch_2_BOOL_val = !switch_2_BOOL_val;
			//std::cout << "Switch 2 detected in my_foot_pedal_USB_stinky: " << keyQString.toStdString() << std::endl;
		}
		else if(keyQString == QString::QString("3"))
		{
			switch_3_BOOL_val = !switch_3_BOOL_val;
			//std::cout << "Switch 3 detected in my_foot_pedal_USB_stinky: " << keyQString.toStdString() << std::endl;
		}
		if(keyQString == QString::QString("4"))
		{
			switch_4_BOOL_val = !switch_4_BOOL_val;
			//std::cout << "Switch 4 detected in my_foot_pedal_USB_stinky: " << keyQString.toStdString() << std::endl;
		}
	keyEvent_mutex->unlock();

	incoming_key_event->ignore();
	QWidget::keyPressEvent(incoming_key_event); //This line is recommended by the QT class documentation.
	
}

void my_foot_pedal_USB_stinky::close(void)
{
    
}

void my_foot_pedal_USB_stinky::timer_update(QTimerEvent *event)
{
	switch_1_label->setText("Switch 1: " + QString::number(switch_1_BOOL_val));
	switch_2_label->setText("Switch 2: " + QString::number(switch_2_BOOL_val));
	switch_3_label->setText("Switch 3: " + QString::number(switch_3_BOOL_val));
	switch_4_label->setText("Switch 4: " + QString::number(switch_4_BOOL_val));

	if(getKeyboardFocusValue() == 0)
	{
		background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(255,200,200)}");
	}
	else
	{
		background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(200,255,200)}");
	}
}

bool my_foot_pedal_USB_stinky::getKeyboardFocusValue(void)
{
	return this->hasFocus();
	//return do_i_have_focus_flag;
}

void my_foot_pedal_USB_stinky::my_setFocus_function(bool val)
{
	if(val == 1)
	{
		this->setFocus();
	}
	else if(val == 0)
	{
		this->clearFocus();
	}
	
	do_i_have_focus_flag = val;
}