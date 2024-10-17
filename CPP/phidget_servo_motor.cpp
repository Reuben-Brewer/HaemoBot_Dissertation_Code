#include "phidget_servo_motor.h"

phidget_servo_motor::phidget_servo_motor(int motor_ID_in)
{
    motor_ID = motor_ID_in;

    int result;
    double curr_pos;
    const char *err;


    //Declare an advanced servo handle
    //CPhidgetAdvancedServoHandle servo = 0;

    //create the advanced servo object
    CPhidgetAdvancedServo_create(&servo);

    //open the device for connections
    CPhidget_open((CPhidgetHandle)servo, -1);

    //get the program to wait for an advanced servo device to be attached
    printf("Waiting for Phidget to be attached....");
    if((result = CPhidget_waitForAttachment((CPhidgetHandle)servo, 10000)))
    {
                    CPhidget_getErrorDescription(result, &err);
                    printf("Problem waiting for attachment: %s\n", err);

    }

    printf("Connected to the servo board \n");

    num_motors = motor_ID;

    //Set up some initial acceleration and velocity values
    CPhidgetAdvancedServo_getPositionMin(servo, motor_ID, &min_pos);
    CPhidgetAdvancedServo_getPositionMax(servo, motor_ID, &max_pos);

    CPhidgetAdvancedServo_getVelocityMin(servo, motor_ID, &min_vel);
    CPhidgetAdvancedServo_getVelocityMax(servo, motor_ID, &max_vel);

    CPhidgetAdvancedServo_getAccelerationMin(servo, motor_ID, &min_accel);
    CPhidgetAdvancedServo_getAccelerationMax(servo, motor_ID, &max_accel);

    for(int i = 0; i < num_motors; i++)
    {
        CPhidgetAdvancedServo_setEngaged(servo, i, 1);
        actual_velocity.push_back(0);
        actual_position.push_back(0);
    }

}

void phidget_servo_motor::setEnabled(int num, bool enabled)
{
  CPhidgetAdvancedServo_setEngaged(servo, num, enabled);
}

void phidget_servo_motor::changePosition(int num, double p)
{
  CPhidgetAdvancedServo_setPosition (servo, num, p);
}

void phidget_servo_motor::changeVelocityLimit(int num, double v)
{
  CPhidgetAdvancedServo_setVelocityLimit(servo, num, v);
}

void phidget_servo_motor::changeAccelerationLimit(int num, double a)
{
  CPhidgetAdvancedServo_setAcceleration(servo, num, a);
}

void phidget_servo_motor::close()
{
    for(int i = 0; i < num_motors; i++)
    {
        CPhidgetAdvancedServo_setEngaged(servo, i, 0);
    }

    CPhidget_close((CPhidgetHandle)servo);
    CPhidget_delete((CPhidgetHandle)servo);
}

int phidget_servo_motor::readPosition(int num)
{
    CPhidgetAdvancedServo_getPosition(servo, num, &actual_position[num]);
    return actual_position[num];
}

double phidget_servo_motor::readVelocity(int num)
{
    CPhidgetAdvancedServo_getPosition(servo, num, &actual_velocity[num]);
    return actual_velocity[num];
}
