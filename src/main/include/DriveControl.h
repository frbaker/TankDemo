#ifndef DRIVECONTROL_H
#define DRIVECONTROL_H

#include <frc/XboxController.h>
#include "DriveTrain.h"    //Header for all of our drive control and motors
#include "RobotAuxilary.h" //Header for robots arm and puncher, and ect
#include "Timer.h"         //Self made simple timer

class DriveControl
{

public:
    DriveControl(DriveTrain *dtobj, RobotAuxilary *auxobj); // Ctor
    ~DriveControl();                                        // Dtor
    void teleopController();                                // Bundles drive train with other robot opertaions

private:
    void tankOperation();                               // Enables drive train operation in tank mode
    void traditionalDrive();                            // Enables drive train operation in single stick mode
    double filterInput(double input, double threshold); // Used to filter controller input to prevent drift
    void pollButtons();                                 // Used to manage button actions on the controllers
    // Members Variables
    // Heap
    DriveTrain *drivebase;             // Variable for holding drive train object
    RobotAuxilary *utilites;           // Variable for holding robot utilities like arm and chrammer
    frc::XboxController *controller_1; // Main Controller
    frc::XboxController *controller_2; // Secondary Controller
    // Stack
    bool is_tank_drive; // variable to hold drive style
    // Timers
    Timer *button_grace_period_timer;
};

#endif // DRIVECONTROL_H