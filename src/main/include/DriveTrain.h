#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <rev/CANSparkMAX.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>

class DriveTrain
{

public:
    DriveTrain();  // Ctor
    ~DriveTrain(); // Dtor
    bool moveToForward(double lpos, double rpos);
    bool moveToBackward(double lpos, double rpos);
    void setSpeed(double ls, double rs);
    double getLeftPosition();
    double getRightPostion();
    double getLeftPower();
    double getRightPower();
    double getAngle();

private:
    void configureMotors(); // Used to configure motors specifically for this robot
    void setZero();         // Used to set encoder positions to zero upon object creation

    rev::CANSparkMax *left_motor_1;
    rev::CANSparkMax *left_motor_2;
    rev::CANSparkMax *right_motor_1;
    rev::CANSparkMax *right_motor_2;

    rev::SparkMaxRelativeEncoder *left_encoder_1;
    rev::SparkMaxRelativeEncoder *left_encoder_2;
    rev::SparkMaxRelativeEncoder *right_encoder_1;
    rev::SparkMaxRelativeEncoder *right_encoder_2;

    ctre::phoenix::sensors::PigeonIMU *gyro;

    const double kerror = 10;
};

#endif // DRIVETRAIN_H