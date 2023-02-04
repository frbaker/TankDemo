#include "DriveControl.h"
#include <algorithm>
#include <cmath>

#define PI 3.141592653589793

/**
 * @brief Construct a new Drive Control:: Drive Control object
 *
 * @param dtobj Takes a pointer to the robot drive train object
 */
DriveControl::DriveControl(DriveTrain *dtobj, RobotAuxilary *auxobj)
{
    controller_1 = new frc::XboxController(0); // Init main controller
    controller_2 = new frc::XboxController(1); // Init secondary controller
    drivebase = dtobj;                         // Get the drivetrain object
    utilites = auxobj;                         // Get the auxilary object
    is_tank_drive = true;                      // Start the robot in tank drive mode
    // Timers
    button_grace_period_timer = new Timer(300); // Debounce time in milliseconds
}

/**
 * @brief Main method to handle robot-joystick interaction during teleop
 *
 */
void DriveControl::teleopController()
{
    pollButtons(); // Continuously check the state of the buttons on the xbox controllers and respond accordingly

    if (is_tank_drive)
    {                    // if the tank drive boolean is true, then use tank dive
        tankOperation(); // Use tank drive style
    }
    else
    {                       // else use traditional drive
        traditionalDrive(); // Use traditional drive style
    }
}

/**
 * @brief Tank drive style implementation
 *
 */
void DriveControl::tankOperation()
{
    // Use tank drive and ramp motor power output by squaring the controller input to form a nice curve
    drivebase->setSpeed(std::pow(filterInput(controller_1->GetLeftY(), 0.175), 3.0), std::pow(filterInput(controller_1->GetRightY(), 0.175), 3.0));
}

/**
 * @brief Traditional drive style implementation
 *
 */
void DriveControl::traditionalDrive()
{
    double y = filterInput(controller_1->GetLeftY(), 0.175);  // Get the overall power value after filtering the deadband
    double x = filterInput(controller_1->GetRightX(), 0.175); // Get the steering value multiplier after filtering the deadband
    double leftpower = 0.;
    double rightpower = 0.;

    if (y > 0.0)
    { // Forward
        leftpower = std::clamp(std::pow(y, 2.0) + std::sin(PI * x / 4), 0.1, 1.0);
        rightpower = std::clamp(std::pow(y, 2.0) + std::sin(PI * x / 4), 0.1, 1.0);
    }
    else if (y < 0.0)
    {                                                        // Reverse
        leftpower = -std::clamp(std::pow(y, 2.0), 0.1, 1.0); // Set negative value for reverse
        rightpower = -std::clamp(std::pow(y, 2.0), 0.1, 1.0);
    }
    else
    { // 0 power, so spin
        leftpower = std::pow(-x, 3.0);
        rightpower = std::pow(x, 3.0);
    }
    drivebase->setSpeed(leftpower, rightpower); // Set power values to the motor
}

/**
 * @brief Takes joystick input and filters out low values that can be caused by joystick drift
 *
 * @param input Data to filter
 * @param threshold Ammount to filter relative to base 0
 * @return double
 */
double DriveControl::filterInput(double input, double threshold)
{
    double tempval = input;

    if (std::abs(tempval) > std::abs(threshold))
    { // if the value is above the threshold, then we just return it.
        return tempval;
    }

    return 0.0; // Return zero if the input from the controller does not reach above the threshold
}

/**
 * @brief Handle button input and mappings
 *
 */
void DriveControl::pollButtons()
{
    // Change drive styles
    if (controller_1->GetBackButton() && controller_1->GetStartButton() && button_grace_period_timer->getTimer())
    {                                   // Swap drive modes if start and back button are pushed simultaneously
        is_tank_drive = !is_tank_drive; // Swap tank drive state
    }

    if (controller_1->GetAButton() & button_grace_period_timer->getTimer())
    {
        // Template for the controller 1 a button
    }

    //CHRAM!!!
    if(controller_2->GetAButton() && button_grace_period_timer->getTimer()){
        utilites->chram();//Punch cube
    }

    //Pinchers
    if(controller_1->GetXButton() && button_grace_period_timer->getTimer()){
        utilites->togglePincher();//Toggle pinching the cube
    }
}

/**
 * @brief Destroy the Drive Control:: Drive Control object
 *
 */
DriveControl::~DriveControl()
{
    // Free memory
    delete controller_1;
    delete controller_2;
    delete button_grace_period_timer;
}