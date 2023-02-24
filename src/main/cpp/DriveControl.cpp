#include "DriveControl.h"
#include <algorithm>
#include <cmath>
#include <iostream>

#define PI 3.141592653589793

/**
 * @brief Construct a new Drive Control:: Drive Control object
 *
 * @param dtobj Takes a pointer to the robot drive train object
 */
DriveControl::DriveControl(DriveTrain *dtobj, RobotAuxilary *auxobj, Vision *camobj)
{
    controller_1 = new frc::XboxController(0); // Init main controller
    controller_2 = new frc::XboxController(1); // Init secondary controller
    drivebase = dtobj;                         // Get the drivetrain object
    utilites = auxobj;                         // Get the auxilary object
    camera = camobj;
    is_tank_drive = false; // Start the robot in tank drive mode
    is_turning = false;    // Start that we are not turning
    turn_mode = 0;         // Start that we are neither intending to turn cw or ccw
    coast_mode = true; //Start in coast mode
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
 * @brief This function manages any teleop functions not handled by drivers
 *
 */
void DriveControl::driveManager()
{
    if (!controller_2->GetAButton())
    {
        utilites->unChram(); // Auto retract the chrammer
    }
}

/**
 * @brief Tank drive style implementation
 *
 */
void DriveControl::tankOperation()
{
    // Use tank drive and ramp motor power output by squaring the controller input to form a nice curve
    if (!is_turning)
    {
        drivebase->setSpeed(std::pow(filterInput(controller_1->GetLeftY(), 0.25), 3.0), std::pow(filterInput(controller_1->GetRightY(), 0.25), 3.0));
    }
}

/**
 * @brief Traditional drive style implementation
 *
 */
void DriveControl::traditionalDrive()
{
    double y = filterInput(controller_1->GetLeftY(), 0.25);  // Get the overall power value after filtering the deadband
    double x = filterInput(controller_1->GetRightX(), 0.25); // Get the steering value multiplier after filtering the deadband
    double leftpower = 0.;
    double rightpower = 0.;

    if (y > 0.0)
    { // Forward
        leftpower = std::clamp(std::pow(y, 2.0) + std::sin(PI * x / 4), 0.1, 1.0);
        rightpower = std::clamp(std::pow(y, 2.0) - std::sin(PI * x / 4), 0.1, 1.0);
    }
    else if (y < 0.0)
    { // Reverse -- set negative values
        leftpower = -std::clamp(std::pow(y, 2.0) + std::sin(PI * x / 4), 0.1, 1.0);
        rightpower = -std::clamp(std::pow(y, 2.0) - std::sin(PI * x / 4), 0.1, 1.0);
    }
    else
    { // 0 power, so spin
        leftpower = -x * .75;
        rightpower = x * .75;
    }
    if (!is_turning)
    {
        drivebase->setSpeed(leftpower, rightpower); // Set power values to the motor
    }
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

    if (filterInput(controller_1->GetLeftY, drift_comp) != 0.0 ||
        filterInput(controller_1->GetLeftX, drift_comp) != 0.0 ||
        filterInput(controller_1->GetRightY, drift_comp) != 0.0 ||
        filterInput(controller_1->GetRightY, drift_comp) != 0.0)
    {
        is_turning = false; // Yield control back to the driver from auto turning or cube alignment
    }


    if(controller_1->GetAButton() && button_grace_period_timer->getTimer()){
        drivebase->setCoastMode();//Set robot to coast mode on A button push
    }

    if(controller_1->GetAButton() && button_grace_period_timer->getTimer()){
        drivebase->setBreakMode();//Set robot to break mode on B button push. Used to hold robot on ramp during endgame
    }


    // Change drive styles
    if (controller_1->GetBackButton() && controller_1->GetStartButton() && button_grace_period_timer->getTimer())
    {                                   // Swap drive modes if start and back button are pushed simultaneously
        is_tank_drive = !is_tank_drive; // Swap tank drive state
    }

    if (controller_1->GetLeftBumper() && button_grace_period_timer->getTimer())
    {
        is_turning = true;
        turn_mode = -1; // We are turning ccw
    }
    else if (controller_1->GetRightBumper() && button_grace_period_timer->getTimer())
    {
        is_turning = true;
        turn_mode = 1; // We are turning cw
    }
    else if (controller_1->GetXButton() && button_grace_period_timer->getTimer())
    {
        is_turning = true;
        turn_mode = 2;
    }

    if (is_turning && turn_mode == 1)
    {
        if (drivebase->relativeTurn(90))
        {
            drivebase->resetFlags();
            is_turning = false;
            turn_mode = 0;
        }
    }
    else if (is_turning && turn_mode == -1)
    {
        if (drivebase->relativeTurn(-90))
        {
            drivebase->resetFlags();
            is_turning = false;
            turn_mode = 0;
        }
    }
    else if (is_turning && turn_mode == 2)
    {
        if (drivebase->relativeTurn(camera->getCurrentYaw()))
        {
            drivebase->resetFlags();
            is_turning = false;
            turn_mode = 0;
        }
    }

    // CHRAM!!!
    if (controller_2->GetAButton() && button_grace_period_timer->getTimer())
    {
        utilites->chram(); // Punch cube
    }

    // Pinchers
    if (controller_2->GetXButton() && button_grace_period_timer->getTimer())
    {
        utilites->togglePincher(); // Toggle pinching the cube
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

/*Graveyard

    //Debug on controller A button
    if (controller_1->GetAButton() && button_grace_period_timer->getTimer())
    {
        std::cout << "Left Position: " << drivebase->getLeftPosition() << std::endl;
        std::cout << "Right Position: " << drivebase->getRightPostion() << std::endl;
        std::cout << "Angle: " << drivebase->getAngle() << std::endl;
    }

*/