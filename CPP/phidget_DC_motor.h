#ifndef PHIDGET_DC_MOTOR_H
#define PHIDGET_DC_MOTOR_H

#ifndef PHIDGET21_H
#define PHIDGET21_H
#include <phidget21.h>
#endif

#include <QObject>
#include <iostream>

#define pi_define 3.141592654

class phidget_DC_motor : public QObject
{
    Q_OBJECT

public:
    phidget_DC_motor(double CPR_in = 0.0);
    void updatePID(void);
    void setEncoder(int pos);

    int actual_position_counts;
    double goal_position_degrees, goal_position_radians;
    double actual_position_degrees, actual_position_radians;
    double actual_velocity_degrees, actual_velocity_radians, actual_velocity_counts;
    double goal_velocity_degrees, goal_velocity_radians, goal_velocity_counts;
    double commandVoltage;

    double kp, kv, ki, CPR;
    double error_sum, error;


public Q_SLOTS:
    void changeVelocity(double v);
    void changeAcceleration(double a);
    void close(void);
    int readEncoder(void);
    double readVelocity(void);

protected:
    CPhidgetMotorControlHandle motoControl;
};

#endif
