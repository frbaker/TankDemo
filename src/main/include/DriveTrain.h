#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <rev/CANSparkMAX.h>
#include <frc/DigitalOutput.h>
#include <frc/AnalogEncoder.h>
#include "DataPacket.h"


#include "ctre/Phoenix.h"

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

  frc::AnalogEncoder *swerve_encoder_1;
  frc::AnalogEncoder *swerve_encoder_2;
  frc::AnalogEncoder *swerve_encoder_3;
  frc::AnalogEncoder *swerve_encoder_4;


  ctre::phoenix::motorcontrol::can::TalonSRX *drive_motor_1;
  ctre::phoenix::motorcontrol::can::TalonSRX *drive_motor_2;
  ctre::phoenix::motorcontrol::can::TalonSRX *drive_motor_3;
  ctre::phoenix::motorcontrol::can::TalonSRX *drive_motor_4;

  ctre::phoenix::motorcontrol::can::TalonSRX *swerve_motor_1;
  ctre::phoenix::motorcontrol::can::TalonSRX *swerve_motor_2;
  ctre::phoenix::motorcontrol::can::TalonSRX *swerve_motor_3;
  ctre::phoenix::motorcontrol::can::TalonSRX *swerve_motor_4;
 

};

#endif // DRIVETRAIN_H