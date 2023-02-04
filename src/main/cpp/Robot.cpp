// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "DriveTrain.h"
#include "RobotAuxilary.h"
#include "DriveControl.h"
#include "Telemetry.h"
#include "Vision.h"

DriveTrain drivetrain;                           // Object to control drive motors
RobotAuxilary utilites;                          // Object to control robot arm and puncher
DriveControl controller(&drivetrain, &utilites); // Create drive control object
Telemetry data(&drivetrain, &utilites);          // Reference our drivetrain object
Vision vision;                                   // Object for interfacing with camera

// Runs once one startup
void Robot::RobotInit()
{
  // Link our drivetrain with our telemetry
  drivetrain.loadTelemetry(data.exportTelemetry());
}
// Put code here to be called constantly regardless of robot state
void Robot::RobotPeriodic()
{
  vision.test();
}

// Put auto startup code here. Runs once on auto start.
void Robot::AutonomousInit()
{
}
// Put main auto code here. Called every 20s during auto.
void Robot::AutonomousPeriodic()
{
  utilites.calibrateArm(); // Calibrate arm encoder
  data.runMetrics();       // Constantly update robot position data
}
// Runs once on teleop start
void Robot::TeleopInit() {}
// Put main code here. Called every 20ms by default.
void Robot::TeleopPeriodic()
{
  controller.teleopController(); // Take input from controllers -- main control during teleop
  controller.driveManager();//Handle any robot functions needed outside of driver input
  data.runMetrics();             // Constantly update robot position data
}

void Robot::DisabledInit() {}     // Not used
void Robot::DisabledPeriodic() {} // Not used

void Robot::TestInit() {}     // Not used
void Robot::TestPeriodic() {} // Not used

void Robot::SimulationInit() {}     // Not used
void Robot::SimulationPeriodic() {} // Not used

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
