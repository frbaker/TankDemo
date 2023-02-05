#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <rev/CANSparkMAX.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>

class DriveTrain
{

public:
    DriveTrain();  // Ctor
    ~DriveTrain(); // Dtor
    void setZero();
    bool absoluteMoveForward(double lpos, double rpos);
    bool absoluteMoveBackward(double lpos, double rpos);
    bool absoluteTurnCW(double ang);
    bool absoluteTurnCCW(double ang);

    bool relativeMoveForward(double lpos, double rpos);
    bool relativeMoveBackward(double lpos, double rpos);

    void setSpeed(double ls, double rs);
    double getLeftPosition();
    double getRightPostion();
    double getLeftPower();
    double getRightPower();
    double getAngle();
    void resetFlags();

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

    bool on_init;
    bool at_position_left;
    bool at_position_right;
    double relative_left_pos_zero;
    double relative_right_pos_zero;
    double relative_ang_zero;
    bool at_angle;


    const double k_angle_error = 5.0;
    const double krevs_per_inch = 1.6146;//Number of revolutions per inch
};

#endif // DRIVETRAIN_H