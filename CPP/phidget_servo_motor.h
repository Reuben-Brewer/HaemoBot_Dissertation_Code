#ifndef PHIDGET_SERVO_MOTOR_H
#define PHIDGET_SERVO_MOTOR_H

#ifndef PHIDGET21_H
#define PHIDGET21_H
#include <phidget21.h>
#endif

#include <QObject>
#include <iostream>
#include <vector>

class phidget_servo_motor : public QObject
{
    Q_OBJECT

public:
    phidget_servo_motor(int motor_ID_in = 0);

    void setEnabled(int num, bool enabled);

    int motor_ID;
    std::vector<double> actual_position, actual_velocity;
    double min_pos, max_pos, min_accel, max_accel, min_vel, max_vel;

    int num_motors;

public Q_SLOTS:
    void changePosition(int num, double p);
    void changeVelocityLimit(int num, double v);
    void changeAccelerationLimit(int num, double a);
    void close(void);
    int readPosition(int num);
    double readVelocity(int num);

protected:
    CPhidgetAdvancedServoHandle servo;
};

#endif
