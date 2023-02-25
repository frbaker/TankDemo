#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include "DriveTrain.h"
#include "RobotAuxilary.h"
#include "Vision.h"
#include "Timer.h"
#include <frc/SmartDashboard/SendableChooser.h>

class Autonomous{

    public:
    Autonomous(DriveTrain* dvtobj, RobotAuxilary* auxobj, Vision* camobj);
    ~Autonomous();
    void manageAuto();
    void defaultAuto();
    void straightForward(double dist);
    void custom1();

    private:
    DriveTrain* drivetrain;
    RobotAuxilary* utilties;
    Vision* camera;
    Timer* print_timer;
    frc::SendableChooser<int>* m_chooser;
    
};

#endif//AUTONOMOUS