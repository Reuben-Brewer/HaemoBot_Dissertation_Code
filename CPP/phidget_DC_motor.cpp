#include "phidget_DC_motor.h"


phidget_DC_motor::phidget_DC_motor(double CPR_in)
{
    CPR = CPR_in;

    int result;
    const char *err;

    //create the motor control object
    CPhidgetMotorControl_create(&motoControl);

    //open the motor control for device connections
    CPhidget_open((CPhidgetHandle)motoControl, -1);

    //get the program to wait for a motor control device to be attached
    std::cout << "Waiting for MotorControl to be attached...." << std::endl;
    if((result = CPhidget_waitForAttachment((CPhidgetHandle)motoControl, 10000)))
    {
        CPhidget_getErrorDescription(result, &err);
        std::cout << "Problem waiting for attachment:" << err << std::endl;
    }

    CPhidgetMotorControl_setAcceleration (motoControl, 0, 0.00);
    CPhidgetMotorControl_setVelocity (motoControl, 0, 0.00);

    commandVoltage = 0;
    error_sum = 0;
}

void phidget_DC_motor::changeVelocity(double v)
{
    CPhidgetMotorControl_setVelocity (motoControl, 0, v);
}

void phidget_DC_motor::changeAcceleration(double a)
{
    CPhidgetMotorControl_setAcceleration(motoControl, 0, a);
}

void phidget_DC_motor::close()
{
    changeVelocity(0);
    CPhidget_close((CPhidgetHandle)motoControl);
    CPhidget_delete((CPhidgetHandle)motoControl);
}

int phidget_DC_motor::readEncoder(void)
{
    CPhidgetMotorControl_getEncoderPosition(motoControl, 0, &actual_position_counts);
    actual_position_degrees = actual_position_counts*360/(4*CPR);
    actual_position_radians = actual_position_degrees*pi_define/180;
    return actual_position_degrees;
}

double phidget_DC_motor::readVelocity(void)
{
    CPhidgetMotorControl_getVelocity(motoControl, 0, &actual_velocity_counts);
    actual_velocity_degrees = actual_velocity_counts*360/(4*CPR);
    actual_velocity_radians = actual_velocity_degrees*pi_define/180;
    return actual_velocity_degrees;
}

void phidget_DC_motor::updatePID(void)
{
    readEncoder();
    readVelocity();

    error = goal_position_degrees - actual_position_degrees;
    error_sum = error_sum + error;
    commandVoltage = kp*(error) + kv*(0.0 - actual_velocity_degrees)  + ki*error_sum;
    if(commandVoltage >= 100)
        commandVoltage = 100;
    if(commandVoltage <= -100)
        commandVoltage = -100;

    changeVelocity(commandVoltage);
}

void phidget_DC_motor::setEncoder(int pos)
{
    CPhidgetMotorControl_setEncoderPosition(motoControl, 0, pos);
}
