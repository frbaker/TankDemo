#include "DriveTrain.h"
#include <cmath>

/**
 * @brief Construct a new Drive Train:: Drive Train object
 *
 */
DriveTrain::DriveTrain()
{
    // Spark motor controller creation for neo brushless motors
    left_motor_1 = new rev::CANSparkMax(4, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    left_motor_2 = new rev::CANSparkMax(5, rev::CANSparkMax::MotorType::kBrushless);
    right_motor_1 = new rev::CANSparkMax(6, rev::CANSparkMax::MotorType::kBrushless);
    right_motor_2 = new rev::CANSparkMax(7, rev::CANSparkMax::MotorType::kBrushless);
    configureMotors();
    // Motor encoder creation for neo brushless motors
    // We have to create a copy of the object created by calling get encoder, which returns an object, not a pointer
    left_encoder_1 = new rev::SparkMaxRelativeEncoder(left_motor_1->GetEncoder());
    left_encoder_2 = new rev::SparkMaxRelativeEncoder(left_motor_2->GetEncoder());
    right_encoder_1 = new rev::SparkMaxRelativeEncoder(right_motor_1->GetEncoder());
    right_encoder_2 = new rev::SparkMaxRelativeEncoder(right_motor_2->GetEncoder());
    // Gyro
    gyro = new ctre::phoenix::sensors::PigeonIMU(16); // Create our gyro object with can ID 16
                                                      // Zero all sensors
    setZero();
    resetFlags(); // Reset movement flags
}

/**
 * @brief Configure the motors to turn in the correct direction relative to robot forward.
 * Used at startup
 *
 */
void DriveTrain::configureMotors()
{
    // Robot specific motor settings
    left_motor_1->SetInverted(true); // Invert left side motors since they are flipped
    left_motor_2->SetInverted(true); // Invert left side motors since they are flipped
    //
    left_motor_2->Follow(*left_motor_1);   // Have to dereference pointer since function is defined as pass by reference, so we need to
    right_motor_2->Follow(*right_motor_1); // pass the address of the object, not the address of the pointer
    setCoastMode();//Set our robot to start in coast mode
}

/**
 * @brief Set all encoders to zero. Used at startup
 *
 */
void DriveTrain::setZero()
{
    // Set the start position of the motor encoders to zero
    left_encoder_1->SetPosition(0.0);
    left_encoder_2->SetPosition(0.0);
    right_encoder_1->SetPosition(0.0);
    right_encoder_2->SetPosition(0.0);
    gyro->SetYaw(0.0); // Set the x angle to 0
}

/**
 * @brief Used after a successful movement command to clear flags and allow movement
 *
 */
void DriveTrain::resetFlags()
{
    on_init = true;
    on_init_level2 = true;
    at_position_left = false;
    at_position_right = false;
    relative_left_pos_zero = 0.0;
    relative_right_pos_zero = 0.0;
    at_angle = false;
    relative_ang_zero = 0.0;
    is_balanced = true;
}

/**
 * @brief Turns toward an absolute angle: CW = +angle, CCW = -angle
 *
 * @param desired_ang angle to turn to
 * @return true angle reached
 * @return false angle not yet reached
 */
bool DriveTrain::absoluteTurn(double desired_ang)
{
    if (getAngle() < desired_ang && !at_angle)
    {
        // Turn clockwise to the angle
        if (std::abs(getAngle() - desired_ang) < k_angle_error)
        {
            setSpeed(-.1, .1); // Set left motor forward and right motor backward
            at_angle = true;
        }
        else if (std::abs(getAngle() - desired_ang) < 15 * k_angle_error)
        {
            setSpeed(-.15, .15); // Set left motor forward and right motor backward
        }
        else
        {
            setSpeed(-.575, .575); // Set left motor forward and right motor backward
        }
    }
    else if (getAngle() > desired_ang && !at_angle)
    { // Turn clockwise to the angle
        // Turn counterclockwise to the angle
        if (std::abs(getAngle() - desired_ang) < k_angle_error)
        {
            setSpeed(.1, -.1); // Set left motor forward and right motor backward
            at_angle = true;
        }
        else if (std::abs(getAngle() - desired_ang) < 15 * k_angle_error)
        {
            setSpeed(.15, -.15); // Set left motor forward and right motor backward
        }
        else
        {
            setSpeed(.575, -.575); // Set left motor forward and right motor backward
        }
    }

    if (at_angle)
    {
        setSpeed(0, 0); // Stop
        at_angle = true;
    }
    return at_angle; // Return if we are at the angle
}



bool DriveTrain::relativeTurn(double desired_ang){
    if(on_init_level2){
        relative_ang_zero = getAngle();//Set our relative angle zero to current robot angle
        on_init_level2 = false;
    }
    return absoluteTurn(relative_ang_zero + desired_ang);
}


bool DriveTrain::balance(){

    if(gyro->GetPitch() > 2.0){
        is_balanced = false;
        if(relativeMoveForward(1.0,1.0)){
            resetFlags();
        }
    }else if(gyro->GetPitch() < -2.0){
        is_balanced = false;
        if(relativeMoveBackward(1.0,1.0)){
            resetFlags();
        }
    }else{
        is_balanced = true;
    }

    return is_balanced;
}


bool DriveTrain::relativeMoveForward(double lpos, double rpos)
{

    if (on_init)
    {
        relative_left_pos_zero = getLeftPosition();  // Save left encoder position on first call
        relative_right_pos_zero = getRightPostion(); // Save right encoder position on first call
        on_init = false;                             // No longer first call
    }

    if (lpos + relative_left_pos_zero > getLeftPosition() && !at_position_left)
    {                             // if desired position is behind current encoder position
        left_motor_1->Set(-0.75); // Left motors forward 75%
    }
    else
    {
        left_motor_1->Set(0.0);
        at_position_left = true;
    }

    if (rpos + relative_right_pos_zero > getRightPostion() && !at_position_right)
    {                              // if desired position is behind current encoder position
        right_motor_1->Set(-0.75); // Right motors forward 75%
    }
    else
    {
        right_motor_1->Set(0.0);
        at_position_right = true;
    }

    return at_position_left & at_position_right;
}

bool DriveTrain::relativeMoveBackward(double lpos, double rpos)
{

    if (on_init)
    {
        relative_left_pos_zero = getLeftPosition();  // Save left encoder position on first call
        relative_right_pos_zero = getRightPostion(); // Save right encoder position on first call
        on_init = false;                             // No longer first call
    }

    if (relative_left_pos_zero - lpos < getLeftPosition() && !at_position_left)
    {                            // if desired position is ahead of current encoder position
        left_motor_1->Set(0.75); // Left motors reverse 75%
    }
    else
    {
        left_motor_1->Set(0.0);
        at_position_left = true;
    }

    if (relative_right_pos_zero - rpos < getRightPostion() && !at_position_right)
    {                             // if desired position is ahead of current encoder position
        right_motor_1->Set(0.75); // Right motors reverse 75%
    }
    else
    {
        right_motor_1->Set(0.0);
        at_position_right = true;
    }

    return at_position_left & at_position_right;
}


void DriveTrain::setCoastMode(){
    left_motor_1->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    right_motor_1->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}


void DriveTrain::setBreakMode(){
    left_motor_1->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    right_motor_1->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

/**
 * @brief Set the speed of the robots left and right motors
 *
 * @param ls Left side motor speed
 * @param rs Right side motor speed
 */
void DriveTrain::setSpeed(double ls, double rs)
{
    // Set the speeds for each side of the robots drive train
    left_motor_1->Set(ls);
    // leftmotor2->Set(ls);//Should be controlled by motor 1
    right_motor_1->Set(rs);
    // rightmotor2->Set(rs);//Should be controlled by motor 1
}

/**
 * @brief Returns the current encoder reading of the left side motors
 *
 * @return double Left encoder position
 */
double DriveTrain::getLeftPosition()
{
    return -(left_encoder_1->GetPosition() + left_encoder_2->GetPosition() / 2);
}

/**
 * @brief Returns the current encoder reading of the right side motors
 *
 * @return double Right encoder position
 */
double DriveTrain::getRightPostion()
{
    return -(right_encoder_1->GetPosition() + right_encoder_2->GetPosition() / 2);
}

/**
 * @brief Returns the current power outpout of the left side motors
 *
 * @return double Power output
 */
double DriveTrain::getLeftPower()
{
    return -(left_motor_1->Get() + left_motor_2->Get() / 2);
}

/**
 * @brief Returns the current power output of the right side motors
 *
 * @return double Power output
 */
double DriveTrain::getRightPower()
{
    return -(right_motor_1->Get() + right_motor_2->Get() / 2);
}

/**
 * @brief Returns the angle of the onboard gyro
 *
 * @return double Rurns the angle of the gyro: cw direction is positive
 */
double DriveTrain::getAngle()
{
    return -(gyro->GetYaw());
}

/**
 * @brief Destroy the Drive Train:: Drive Train object
 *
 */
DriveTrain::~DriveTrain()
{
    // Motors
    delete left_motor_1;
    delete left_motor_2;
    delete right_motor_1;
    delete right_motor_2;
    // Encoders
    delete left_encoder_1;
    delete left_encoder_2;
    delete right_encoder_1;
    delete right_encoder_2;
    // Gyro
    delete gyro;
}

/*Graveyard

bool DriveTrain::absoluteMoveForward(double lpos, double rpos)
{
    if (lpos > getLeftPosition() && !at_position_left)
    {                             // if desired position is behind current encoder position
        left_motor_1->Set(-0.25); // Left motors forward 75%
    }
    else
    {
        left_motor_1->Set(0.0);
        at_position_left = true;
    }

    if (rpos > getRightPostion() && !at_position_right)
    {                              // if desired position is behind current encoder position
        right_motor_1->Set(-0.25); // Right motors forward 75%
    }
    else
    {
        right_motor_1->Set(0.0);
        at_position_right = true;
    }

    return at_position_left & at_position_right;
}

bool DriveTrain::absoluteMoveBackward(double lpos, double rpos)
{
    if (lpos < getLeftPosition() && !at_position_left)
    {                            // if desired position is ahead of current encoder position
        left_motor_1->Set(0.25); // Left motors reverse 75%
    }
    else
    {
        left_motor_1->Set(0.0);
        at_position_left = true;
    }

    if (rpos < getRightPostion() && !at_position_right)
    {                             // if desired position is ahead of current encoder position
        right_motor_1->Set(0.25); // Right motors reverse 75%
    }
    else
    {
        right_motor_1->Set(0.0);
        at_position_right = true;
    }

    return at_position_left & at_position_right;
}

*/