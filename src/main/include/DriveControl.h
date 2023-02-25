#ifndef DRIVECONTROL_H
#define DRIVECONTROL_H

#include <frc/XboxController.h>
#include "DriveTrain.h"    //Header for all of our drive control and motors
#include "RobotAuxilary.h" //Header for robots arm and puncher, and ect
#include "Vision.h"
#include "Timer.h"         //Self made simple timer

class DriveControl
{

public:
    DriveControl(DriveTrain *dtobj, RobotAuxilary *auxobj, Vision *camobj); // Ctor
    ~DriveControl();                                        // Dtor
    void teleopController();                                // Bundles drive train with other robot opertaions
    void driveManager();                                    // Handles any robot functionality during teleop that is not controlled by drivers

private:
    void tankOperation();                               // Enables drive train operation in tank mode
    void traditionalDrive();                            // Enables drive train operation in single stick mode
    double filterInput(double input, double threshold); // Used to filter controller input to prevent drift
    void pollButtons();                                 // Used to manage button actions on the controllers
    // Members Variables
    // Heap
    DriveTrain *drivebase;             // Variable for holding drive train object
    RobotAuxilary *utilites;           // Variable for holding robot utilities like arm and chrammer
    Vision *camera;//Variable for accessing robo cam data
    frc::XboxController *controller_1; // Main Controller
    frc::XboxController *controller_2; // Secondary Controller
    double drift_comp;
    // Stack
    bool is_tank_drive; // variable to hold drive style
    bool is_turning;
    int turn_mode;
    bool coast_mode;//Holds whether the motors should engage breaking or coast mode
    // Timers
    Timer *button_grace_period_timer;
};

#endif // DRIVECONTROL_H