#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "ctre/Phoenix.h"
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"
#include <ctre/phoenix/sensors/PigeonIMU.h>

#include <frc/DigitalOutput.h>
#include <frc/AnalogEncoder.h>
#include <frc2/command/PIDSubsystem.h>

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

    void Drive(double x1, double y1, double x2);
    void WheelDrive (int angleMotor, int speedMotor, int encoder);
private:
    void configureMotors(); // Used to configure motors specifically for this robot

/*frc2::PIDController pid{dc::kStabilizationP, dc::kStabilizationI, dc::kStabilizationD}};  */
TalonSRX *angleMotor;
TalonSRX *speedMotor;
frc::AnalogEncoder *imu;
frc2::PIDController *pidController;
TalonSRX *front_left_drive;
TalonSRX *front_left_swerve;
TalonSRX *front_right_drive;
TalonSRX *front_right_swerve;

TalonSRX *back_left_drive;
TalonSRX *back_left_swerve;
TalonSRX *back_right_drive;
TalonSRX *back_right_swerve;

frc::AnalogEncoder *front_left_encoder;
frc::AnalogEncoder *front_right_encoder;
frc::AnalogEncoder *back_left_encoder;
frc::AnalogEncoder *back_right_encoder;

PigeonIMU *gyro;

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