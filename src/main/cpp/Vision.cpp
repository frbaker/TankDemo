#include "Vision.h"

#include <iostream>

Vision::Vision()
{
    camera = new photonlib::PhotonCamera("OV5647");
    result = new photonlib::PhotonPipelineResult;
    target = new photonlib::PhotonTrackedTarget;
}

void Vision::test()
{
    *result = camera->GetLatestResult();
    bool hasTargets = result->HasTargets();
    if (hasTargets)
    {
        *target = result->GetBestTarget();
        // Get information from target.
        double yaw = target->GetYaw();
        std::cout<<"Yaw: "<<yaw<<std::endl;
        double pitch = target->GetPitch();
        double area = target->GetArea();
        double skew = target->GetSkew();
        int targetID = target->GetFiducialId();
        std::cout<<"ID: "<<targetID<<std::endl;
        double poseAmbiguity = target->GetPoseAmbiguity();
        units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
            CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH,
            units::degree_t{target->GetPitch()});

        // Now Conner can do stuff
    }
}

/**
 * @brief Destroy the Vision:: Vision object
 *
 */
Vision::~Vision()
{
    delete camera;
    delete result;
    delete target;
}
