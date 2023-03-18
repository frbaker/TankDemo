#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <rev/CANSparkMAX.h>
#include "ctre/Phoenix.h"
#include <ctre/phoenix/sensors/PigeonIMU.h>

#include <frc/DigitalOutput.h>
#include <frc/AnalogEncoder.h>


class DriveTrain
{

public:
    DriveTrain();  // Ctor
    ~DriveTrain(); // Dtor
    void setZero();
    bool absoluteTurn(double desired_ang);
    bool relativeTurn(double desired_ang);
    bool relativeMoveForward(double lpos, double rpos);
    bool relativeMoveBackward(double lpos, double rpos);
    bool balance();
    void setCoastMode();
    void setBreakMode();
    void setSpeed(double ls, double rs);
    double getLeftPosition();
    double getRightPostion();
    double getLeftPower();
    double getRightPower();
    double getAngle();
    void resetFlags();

private:
    void configureMotors(); // Used to configure motors specifically for this robot
/*
    rev::CANSparkMax *left_motor_1;
    rev::CANSparkMax *left_motor_2;
    rev::CANSparkMax *front_right_drive;
    rev::CANSparkMax *front_right_swerve;

    rev::SparkMaxRelativeEncoder *front_left_encoder;
    rev::SparkMaxRelativeEncoder *front_right_encoder;
    rev::SparkMaxRelativeEncoder *back_left_encoder;
    rev::SparkMaxRelativeEncoder *back_right_encoder;
*/
ctre::phoenix::motorcontrol::can::TalonSRX *front_left_drive;
ctre::phoenix::motorcontrol::can::TalonSRX *front_left_swerve;
ctre::phoenix::motorcontrol::can::TalonSRX *front_right_drive;
ctre::phoenix::motorcontrol::can::TalonSRX *front_right_swerve;

ctre::phoenix::motorcontrol::can::TalonSRX *back_left_drive;
ctre::phoenix::motorcontrol::can::TalonSRX *back_left_swerve;
ctre::phoenix::motorcontrol::can::TalonSRX *back_right_drive;
ctre::phoenix::motorcontrol::can::TalonSRX *back_right_swerve;

frc::AnalogEncoder *front_left_encoder;
frc::AnalogEncoder *front_right_encoder;
frc::AnalogEncoder *back_left_encoder;
frc::AnalogEncoder *back_right_encoder;




    ctre::phoenix::sensors::PigeonIMU *gyro;

    bool on_init;
    bool on_init_level2;
    bool at_position_left;
    bool at_position_right;
    double relative_left_pos_zero;
    double relative_right_pos_zero;
    double relative_ang_zero;
    bool at_angle;
    bool is_balanced;

    const double k_angle_error = 1.0;
    const double krevs_per_inch = 1.6146; // Number of revolutions per inch
};

#endif // DRIVETRAIN_H

/*Graveyard
 bool absoluteMoveForward(double lpos, double rpos);
    bool absoluteMoveBackward(double lpos, double rpos);


*/