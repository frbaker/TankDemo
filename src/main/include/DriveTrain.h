#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

// #include <ctre/phoenixpro/TalonFX.hpp>
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
    bool moveTo(double lpos, double rpos);
    bool snapshotMoveTo(double lpos, double rpos, double lspd, double rspd);
    void setSpeed(double ls, double rs);
    double *getSpeeds();                     // returns the current speeds of each motor
    double *getPositions();                  // Returns the current encoder positions for each motor encoder
    void loadTelemetry(SparkMaxPacket *dta); // Get the pointer for the telemetry data packet
    void updateTelemetry();                  // Fills the telemetry struct with new data

private:
    void configureMotors(); // Used to configure motors specifically for this robot
    void setZero();         // Used to set encoder positions to zero upon object creation

    rev::SparkMaxRelativeEncoder *left_encoder_1;
    rev::SparkMaxRelativeEncoder *left_encoder_2;
    rev::SparkMaxRelativeEncoder *right_encoder_1;
    rev::SparkMaxRelativeEncoder *right_encoder_2;

    SparkMaxPacket *telemetry_link; // Used to hold pointer to telemetry link

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