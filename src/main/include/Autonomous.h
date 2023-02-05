#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include <frc/smartdashboard/SendableChooser.h>
#include "DriveTrain.h"
#include "RobotAuxilary.h"
#include "Telemetry.h"

class Autonomous
{

public:
    Autonomous(DriveTrain* dvtobj, RobotAuxilary* auxobj);
    ~Autonomous();
    void runAuto();
    void test();

private:
    void defaultAction();
    void straightPath(double dist);

    frc::SendableChooser<int> *m_auto_picker; // Object pointer for setting up auto selection
    DriveTrain* drivetrain;//Pointer to drivetrain
    RobotAuxilary* utilities;//Pointer to robot utilites

    double m_relative_left;//Relative placeholder positions for encoders
    double m_relative_right;//Relative placeholder positions for encoders
};

#endif // AUTONOMOUS_H