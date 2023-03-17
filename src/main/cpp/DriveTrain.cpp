#include "DriveTrain.h"
#include "ctre/Phoenix.h"

/**
 * @brief Construct a new Drive Train:: Drive Train object
 *
 */
DriveTrain::DriveTrain()
{
    swerve_encoder_1 = new frc::AnalogEncoder(1);
    swerve_encoder_2 = new frc::AnalogEncoder(2);
    swerve_encoder_3 = new frc::AnalogEncoder(3);
    swerve_encoder_4 = new frc::AnalogEncoder(4);

    drive_motor_1 = new ctre::phoenix::motorcontrol::can::TalonSRX(4);
    drive_motor_2 = new ctre::phoenix::motorcontrol::can::TalonSRX(5);
    drive_motor_3 = new ctre::phoenix::motorcontrol::can::TalonSRX(6);
    drive_motor_4 = new ctre::phoenix::motorcontrol::can::TalonSRX(7);

    swerve_motor_1 = new ctre::phoenix::motorcontrol::can::TalonSRX(8);
    swerve_motor_2 = new ctre::phoenix::motorcontrol::can::TalonSRX(9);
    swerve_motor_3 = new ctre::phoenix::motorcontrol::can::TalonSRX(10);
    swerve_motor_4 = new ctre::phoenix::motorcontrol::can::TalonSRX(11);


    configureMotors();
    // Motor encoder creation for neo brushless motors
    // We have to create a copy of the object created by calling get encoder, which returns an object, not a pointer

    //left_encoder_1 = new rev::SparkMaxRelativeEncoder(left_motor_1->GetEncoder());
    //left_encoder_2 = new rev::SparkMaxRelativeEncoder(left_motor_2->GetEncoder());
    //right_encoder_1 = new rev::SparkMaxRelativeEncoder(right_motor_1->GetEncoder());

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
   // left_motor_1->SetInverted(true); // Invert left side motors since they are flipped
    //left_motor_2->SetInverted(true); // Invert left side motors since they are flipped
    //
    //left_motor_2->Follow(*left_motor_1);   // Have to dereference pointer since function is defined as pass by reference, so we need to
    //right_motor_2->Follow(*right_motor_1); // pass the address of the object, not the address of the pointer
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
    at_position_left = false;
    at_position_right = false;
    relative_left_pos_zero = 0.0;
    relative_right_pos_zero = 0.0;
    relative_ang_zero = 0.0;
    at_angle = false;
}

bool DriveTrain::absoluteMoveForward(double lpos, double rpos)
{
    if (lpos > getLeftPosition() && !at_position_left)
    {                             // if desired position is behind current encoder position
        //left_motor_1->Set(-0.75); // Left motors reverse 75%
    }
    else if (lpos > telemetry_link->left_position)
    {                            // if desired position is ahead of current encoder position
        //left_motor_1->Set(0.75); // Left motors forward 75%

    }

    if (rpos > getRightPostion() && !at_position_right)
    {                              // if desired position is behind current encoder position
        //right_motor_1->Set(-0.75); // Right motors reverse 75%

    }

    if (rpos < getRightPostion() && !at_position_right)
    {                             // if desired position is ahead of current encoder position
        //right_motor_1->Set(0.75); // Right motors forward 75%
    }

    return at_position_left & at_position_right;
}

bool DriveTrain::absoluteTurnCW(double ang)
{
    if (ang > getAngle() - k_angle_error && ang < getAngle() + k_angle_error)
    {                   // Turn clockwise to the angle
        setSpeed(0, 0); // Stop
        at_angle = true;
    }
    else
    {
        setSpeed(-.575, .575); // Set left motor forward and right motor backward
    }
    return at_angle; // Return if we are at the angle
}

bool DriveTrain::absoluteTurnCCW(double ang)
{
    if (ang > getAngle() - k_angle_error || ang < getAngle() + k_angle_error)
    {                          // Turn clockwise to the angle
        setSpeed(.575, -.575); // Set left motor forward and right motor backward
    }
    else
    {
        setSpeed(0, 0); // Stop
        at_angle = true;
    }
    return at_angle; // Return if we are at the angle
}


bool DriveTrain::relativeMoveForward(double lpos, double rpos){
    
    if(on_init){
        relative_left_pos_zero = getLeftPosition();//Save left encoder position on first call
        relative_right_pos_zero = getRightPostion();//Save right encoder position on first call
        on_init = false;//No longer first call
    }

    if (lpos + relative_left_pos_zero > getLeftPosition() && !at_position_left)
    {                             // if desired position is behind current encoder position
        left_motor_1->Set(-0.25); // Left motors forward 75%
    }
    else
    {
        left_motor_1->Set(0.0);
        at_position_left = true;
    }

    if (rpos + relative_right_pos_zero > getRightPostion() && !at_position_right)
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

bool DriveTrain::relativeMoveBackward(double lpos, double rpos){
    
    if(on_init){
        relative_left_pos_zero = getLeftPosition();//Save left encoder position on first call
        relative_right_pos_zero = getRightPostion();//Save right encoder position on first call
        on_init = false;//No longer first call
    }
    
    if (relative_left_pos_zero - lpos < getLeftPosition() && !at_position_left)
    {                            // if desired position is ahead of current encoder position
        left_motor_1->Set(0.25); // Left motors reverse 75%
    }
    else
    {
        left_motor_1->Set(0.0);
        at_position_left = true;
    }

    if (relative_right_pos_zero - rpos < getRightPostion() && !at_position_right)
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





/**
 * @brief Set the speed of the robots left and right motors
 *
 * @param ls Left side motor speed
 * @param rs Right side motor speed
 */
void DriveTrain::setSpeed(double ls, double rs)
{
    drive_motor_1->Set(ControlMode::PercentOutput,0.0);
    drive_motor_2->Set(ControlMode::PercentOutput,0.0);
    drive_motor_3->Set(ControlMode::PercentOutput,0.0);
    drive_motor_4->Set(ControlMode::PercentOutput,0.0);
    
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
    // Update motor power
    //telemetry_link->left_motor_power = left_motor_1->Get();   // Don't average since second motor is just a follower
    //telemetry_link->right_motor_power = right_motor_1->Get(); // Don't average since second motor is just a follower
    // Update encoder positions
    telemetry_link->left_position = left_encoder_1->GetPosition() * left_encoder_2->GetPosition() / 2;    // Average the left side motor values and
    telemetry_link->right_position = right_encoder_1->GetPosition() * right_encoder_2->GetPosition() / 2; // Average the left side motor values

}

/**
 * @brief Destroy the Drive Train:: Drive Train object
 *
 */
DriveTrain::~DriveTrain()
{
    // Motors
    delete drive_motor_1;
    delete drive_motor_2;
    delete drive_motor_3;
    delete drive_motor_4;

    delete swerve_motor_1;
    delete swerve_motor_2;
    delete swerve_motor_3;
    delete swerve_motor_4;


    // Encoders
    delete swerve_encoder_1;
    delete swerve_encoder_2;
    delete swerve_encoder_3;
    delete swerve_encoder_4;


    delete left_encoder_1;
    delete left_encoder_2;
    delete right_encoder_1;
    delete right_encoder_2;
    // Gyro
    delete gyro;
}