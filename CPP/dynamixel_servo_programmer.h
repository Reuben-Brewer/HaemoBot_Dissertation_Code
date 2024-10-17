#ifndef DYNAMIXEL_SERVO_PROGRAMMER_H
#define DYNAMIXEL_SERVO_PROGRAMMER_H

#include "slider_with_box.h"
#include <string>
#include <vector>
#include <QRadioButton>
#include <QPushButton>
#include "rlserial.h"
#include "data_plot.h"
#include <QMutex>
#include "print_labeled_spinbox.h"
#include <QLabel>

class dynamixel_servo_programmer: public QWidget
{
    Q_OBJECT
public:
    dynamixel_servo_programmer(QWidget *parent, QString name_in = "");
    void timer_update(QTimerEvent *e);
	double readPosVel(int servoID);
	void move_servo(int servoID, double goal_pos, double goal_vel);
	void setServoId(void);
	void setServoBaud(void);
	void setServoCompliance(void);
	void setServoStatusReturnLevel(int return_level);
	void send_serial_and_selfread(std::vector<unsigned int> message_vec);
	void toggleLED(int servoID, bool led_status);
	void close(void);

	std::string name;
	int servo_ID, com_open_result;
	int comPortServoResult;

	unsigned int target_servo_ID, new_servo_ID, target_baud, new_baud;
	unsigned int CW_compliance_margin, CCW_compliance_margin, CCW_compliance_slope, CW_compliance_slope;

	print_labeled_spinbox *servo_target_ID_spinbox, *servo_new_ID_spinbox, *servo_compliance_spinbox, *servo_target_baud_spinbox, *servo_new_baud_spinbox;
	QPushButton *reset_servo_button, *program_servo_button, *connect_serial_button;
	rlSerial *comPortServo;
	QLabel *background_box;


private Q_SLOTS:
	void resetServoToFactory(void);
	void programServo(void);
	void connect_serial(void);
};



#endif
