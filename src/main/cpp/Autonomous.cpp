#include "Autonomous.h"
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * @brief Construct a new Autonomous:: Autonomous object
 *
 */
Autonomous::Autonomous(DriveTrain *dvtobj, RobotAuxilary *auxobj)
{
    drivetrain = dvtobj;
    utilities = auxobj;
    m_auto_picker = new frc::SendableChooser<int>(); // Create memory for our chooser
    m_auto_picker->SetDefaultOption("Default", 0);
    m_auto_picker->AddOption("Straight Forward", 1);

    frc::SmartDashboard::PutData("AUTO MODES", m_auto_picker);

    m_relative_left = 0.0;
    m_relative_right = 0.0;
}

void Autonomous::test()
{
    drivetrain->setSpeed(-0.1, -0.1);
}

void Autonomous::runAuto()
{
    switch (m_auto_picker->GetSelected())
    {
    case 1:
        straightPath(20);
        break;
    case 2:
    case 3:
    default:
        defaultAction(); // Call default action
    }
}

/**
 * @brief Default action for autonomous
 *
 */
void Autonomous::defaultAction()
{

    static int step_cnt = 0;
    switch (step_cnt)
    {
    case 0:
        drivetrain->moveTo(-12.8, -12.8);
        step_cnt++;
        break;
    case 1:
        drivetrain->turnTo(-40);
        step_cnt++;
        break;
    case 2:
        drivetrain->moveTo(-50.1, -80.1);
        step_cnt++;
        break;
    case 3:
        drivetrain->turnTo(-1);
        step_cnt++;
        break;
    case 4:
        drivetrain->moveTo(-321.0, -321.0);
        step_cnt++;
        break;
    default:
        break;
    }
}

/**
 * @brief Moves the robot straight forward until a set distance, then stops
 *
 * @param dist How far in feet the robot should move forward
 */
void Autonomous::straightPath(double dist)
{
    drivetrain->moveTo(dist, dist);
}

Autonomous::~Autonomous()
{
    delete m_auto_picker; // Free
}