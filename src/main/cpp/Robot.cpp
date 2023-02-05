// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "DriveTrain.h"
#include "RobotAuxilary.h"
#include "Autonomous.h"
#include "DriveControl.h"
#include "Vision.h"

DriveTrain *drivetrain;   // Object to control drive motors
RobotAuxilary *utilites;  // Object to control robot arm and puncher
Autonomous *auto_manager; // Object to control robot during autonomous
DriveControl *controller; // Create drive control object
Vision *cam;              // Object for interfacing with camera

// Runs once one startup
void Robot::RobotInit()
{
  drivetrain = new DriveTrain();
  utilites = new RobotAuxilary();
  auto_manager = new Autonomous(drivetrain, utilites);
  controller = new DriveControl(drivetrain, utilites);
  cam = new Vision();
}
// Put code here to be called constantly regardless of robot state
void Robot::RobotPeriodic()
{
  cam->test();
}

// Put auto startup code here. Runs once on auto start.
void Robot::AutonomousInit()
{
  drivetrain->setZero();//Set encoder and gyro to 0
}
// Put main auto code here. Called every 20s during auto.
void Robot::AutonomousPeriodic()
{
  // utilites.calibrateArm(); // Calibrate arm encoder -- Runs immediately, but does not continue to run after its finished
  //auto_manager->runAuto(); // Run our selected auto program
  //drivetrain->setSpeed(-0.1,-0.1);
  auto_manager->runAuto();
}
// Runs once on teleop start
void Robot::TeleopInit() {}
// Put main code here. Called every 20ms by default.
void Robot::TeleopPeriodic()
{
  controller->teleopController(); // Take input from controllers -- main control during teleop
  controller->driveManager();     // Handle any robot functions needed outside of driver input
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
