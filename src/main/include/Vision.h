#ifndef VISION_H
#define VISION_H

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>

#include <frc/controller/PIDController.h>
#include <units/angle.h>
#include <units/length.h>

class Vision
{
public:
    Vision();
    void run();
    double getCurrentYaw();
    ~Vision();

private:
    photonlib::PhotonCamera *camera;
    photonlib::PhotonPipelineResult *result;
    photonlib::PhotonTrackedTarget *target;

    double current_yaw;
    double stored_yaw;

    // TODO - change all of these to appropriate settings once we have camera mounted
    const units::meter_t CAMERA_HEIGHT = 22_in;    // camera height on robot
    const units::meter_t TARGET_HEIGHT = 5_ft;     // height of target
    const units::radian_t CAMERA_PITCH = -10_deg;    // angle between horizontal and camera angle
    const units::meter_t GOAL_RANGE_METERS = 2_ft; // distance from target we want to be
    // const double P_GAIN = 0.1; //are we using frc pid - or your own version?
    // const double D_GAIN = 0.0;
    // frc2::PIDController controller{P_GAIN, 0.0, D_GAIN};
};

#endif // VISION_H
