// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "DriveTrain.h"
#include "RobotAuxilary.h"
#include "Autonomous.h"
#include "DriveControl.h"
#include "Vision.h"

DriveTrain drivetrain;                           // Object to control drive motors
RobotAuxilary utilites;                          // Object to control robot arm and puncher
Vision vision;                                   // Object for interfacing with camera
Autonomous auto_manager(&drivetrain,&utilites);                         // Object to control autonomous
DriveControl controller(&drivetrain, &utilites, &vision); // Create drive control object

// Runs once one startup
void Robot::RobotInit()
{
  // Link our drivetrain with our telemetry
  utilites.togglePincher();//Toggle our pincher on start
}
// Put code here to be called constantly regardless of robot state
void Robot::RobotPeriodic()
{

}

// Put auto startup code here. Runs once on auto start.
void Robot::AutonomousInit()
{
  drivetrain.setZero();//Set encoder and gyro to 0
}
// Put main auto code here. Called every 20s during auto.
void Robot::AutonomousPeriodic()
{
  auto_manager.manageAuto();//Mange what auto is running
}
// Runs once on teleop start
void Robot::TeleopInit() {}
// Put main code here. Called every 20ms by default.
void Robot::TeleopPeriodic()
{
  controller.teleopController(); // Take input from controllers -- main control during teleop
  controller.driveManager();     // Handle any robot functions needed outside of driver input
  vision.run();
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
