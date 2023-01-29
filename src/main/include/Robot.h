// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <photonlib/PhotonCamera.h>

#include <frc/TimedRobot.h>
#include <frc/controller/PIDController.h>

#include <units/angle.h>
#include <units/length.h>


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

  private:
  //TODO - change all of these to appropriate settings once we have camera mounted
  const units::meter_t CAMERA_HEIGHT = 24_in; //camera height on robot
  const units::meter_t TARGET_HEIGHT = 5_ft; //height of target
  const units::radian_t CAMERA_PITCH = 0_deg; //angle between horizontal and camera angle
  const units::meter_t GOAL_RANGE_METERS = 3_ft; //distance from target we want to be
  //const double P_GAIN = 0.1; //are we using frc pid - or your own version?
  //const double D_GAIN = 0.0;
  //frc2::PIDController controller{P_GAIN, 0.0, D_GAIN}; 

  photonlib::PhotonCamera camera{"photonvision"}; //change to name configured in the UI
};
