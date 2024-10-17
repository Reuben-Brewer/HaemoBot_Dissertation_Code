#ifndef _MY_FOOT_PEDAL_USB_STINK_H
#define _MY_FOOT_PEDAL_USB_STINK_H 1

#include <string>
#include <iostream>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QLabel>
#include <QMutex>
#include "robot_defines.h"

class my_foot_pedal_USB_stinky: public QWidget
{
    Q_OBJECT

public:

	my_foot_pedal_USB_stinky(QWidget *parent = 0, std::string label_text = "");
    void mousePressEvent(QMouseEvent *incoming_mouse_event);
	void keyPressEvent(QKeyEvent *incoming_key_event);
	void timer_update(QTimerEvent *e);
    void close(void);
	bool getKeyboardFocusValue(void);
	void my_setFocus_function(bool val);
	
	QMutex *keyEvent_mutex;
	QString keyQString;
    QLabel *background_box, *box_title, *switch_1_label, *switch_2_label, *switch_3_label, *switch_4_label;
	bool switch_1_BOOL_val, switch_2_BOOL_val, switch_3_BOOL_val, switch_4_BOOL_val;
	bool do_i_have_focus_flag;

public Q_SLOTS:

protected:
   
private:




};

#endif
