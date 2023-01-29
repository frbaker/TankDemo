#include "Vision.h"

#include "stdio.h"
#include <iostream>

Vision::Vision()
{
   /* camera = new photonlib::PhotonCamera("photonvision");
    result = new photonlib::PhotonPipelineResult;
    target = new photonlib::PhotonTrackedTarget;*/
}

void Vision::test()
{
    printf("Entered test\n");
    photonlib::PhotonCamera camera("fido");
    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    std::cout<<result.HasTargets()<<std::endl;
    printf("Please\n");
    bool hasTargets = result.HasTargets();
    if (hasTargets)
    {
        photonlib::PhotonTrackedTarget target = result.GetBestTarget();
        // Get information from target.
        double yaw = target.GetYaw();
        double pitch = target.GetPitch();
        double area = target.GetArea();
        double skew = target.GetSkew();
        printf("%lf\n",skew);
        int targetID = target.GetFiducialId();
        double poseAmbiguity = target.GetPoseAmbiguity();
        units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
            CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH,
            units::degree_t{target.GetPitch()});

        // Now Conner can do stuff
    }
}

/**
 * @brief Destroy the Vision:: Vision object
 *
 */
Vision::~Vision()
{
   /* delete camera;
    delete result;
    delete target;*/
}
