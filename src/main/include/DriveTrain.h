#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <rev/CANSparkMAX.h>

class DriveTrain
{

public:
    DriveTrain();  // Ctor
    ~DriveTrain(); // Dtor
    void setZero();         // Used to set encoder positions to zero upon object creation

    bool moveTo(double lpos, double rpos);
    bool turnTo(double ang);
    void setSpeed(double ls, double rs);
    double getLeftPosition();
    double getRightPosition();
    double getLeftPower();
    double getRightPower();
    double getAngle();
    double toInches();

private:
    void configureMotors(); // Used to configure motors specifically for this robot
    

    rev::CANSparkMax *left_motor_1;
    rev::CANSparkMax *left_motor_2;
    rev::CANSparkMax *right_motor_1;
    rev::CANSparkMax *right_motor_2;

    rev::SparkMaxRelativeEncoder *left_encoder_1;
    rev::SparkMaxRelativeEncoder *left_encoder_2;
    rev::SparkMaxRelativeEncoder *right_encoder_1;
    rev::SparkMaxRelativeEncoder *right_encoder_2;

    ctre::phoenix::sensors::PigeonIMU *gyro;

    const double toInch = 0.62136; // Conversion factor for encoders
};

#endif // DRIVETRAIN_H