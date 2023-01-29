// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "DriveControl.h"
#include "Telemetry.h"
#include <photonlib/PhotonUtils.h>

DriveTrain drivetrain;                // Object to control drive motors
DriveControl controller(&drivetrain); // Create drive control object
Telemetry data(&drivetrain);          // Reference our drivetrain object

void Robot::RobotInit()
{
  // Link our drivetrain with our telemetry
  drivetrain.loadTelemetry(data.exportTelemetry());
} // Runs once one startup
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit()
{
  // Put auto startup code here. Runs once on auto start.

} // Runs once on auto start
void Robot::AutonomousPeriodic()
{
  // Put main auto code here. Called every 20s during auto.
  
  
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  bool hasTargets = result.HasTargets();
  if (hasTargets){
    photonlib::PhotonTrackedTarget target = result.GetBestTarget(); 
    // Get information from target.
    double yaw = target.GetYaw();
    double pitch = target.GetPitch();
    double area = target.GetArea();
    double skew = target.GetSkew();
    int targetID = target.GetFiducialId();
    double poseAmbiguity = target.GetPoseAmbiguity();
    units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
      CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH,
      units::degree_t{target.GetPitch()});

    //Now Conner can do stuff
  }

  data.runMetrics(); // Constantly update robot position data
}

void Robot::TeleopInit() {} // Runs once on teleop start
void Robot::TeleopPeriodic()
{
  // Put main code here. Called every 20ms by default.
  controller.teleopController(); // Take input from controllers -- main control during teleop
  data.runMetrics();             // Constantly update robot position data

} // Put main teleop control code here

void Robot::DisabledInit() {}     // Garbage
void Robot::DisabledPeriodic() {} // Garbage

void Robot::TestInit() {}     // Garbage
void Robot::TestPeriodic() {} // Garbage

void Robot::SimulationInit() {}     // Garbage
void Robot::SimulationPeriodic() {} // Garbage

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
