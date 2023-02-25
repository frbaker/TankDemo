#include "Vision.h"

#include <iostream>
#include <cmath>

Vision::Vision()
{
    camera = new photonlib::PhotonCamera("OV5647");
    result = new photonlib::PhotonPipelineResult;
    target = new photonlib::PhotonTrackedTarget;
    current_yaw = 0.0;
    stored_yaw = current_yaw;
}

void Vision::run()
{
    *result = camera->GetLatestResult();
    bool hasTargets = result->HasTargets();
    if (hasTargets)
    {
        *target = result->GetBestTarget();
        // Get information from target.
        current_yaw = target->GetYaw();
        if(std::abs(stored_yaw) >= std::abs(current_yaw)+2 || std::abs(stored_yaw) <= std::abs(current_yaw)-2){
            stored_yaw = current_yaw;
        }
        // std::cout<<"Yaw: "<<yaw<<std::endl;
        // double pitch = target->GetPitch();
        // double area = target->GetArea();
        // double skew = target->GetSkew();
        // int targetID = target->GetFiducialId();
        // std::cout<<"ID: "<<targetID<<std::endl;
        // double poseAmbiguity = target->GetPoseAmbiguity();
        // units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
        //    CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH,
        //     units::degree_t{target->GetPitch()});

        // Now Conner can do stuff
    }
}

double Vision::getCurrentYaw()
{
    return current_yaw;
}

double Vision::getStoredYaw(){
    return stored_yaw;
}

void Vision::resetFlags(){
    stored_yaw = current_yaw;
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
