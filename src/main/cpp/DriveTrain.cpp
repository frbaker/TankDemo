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
    setZero();
    // Gyro
    gyro = new ctre::phoenix::sensors::PigeonIMU(16); // Create our gyro object with can ID 16
    gyro->SetYaw(0.0);                                // Set the x angle to 0
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
}

bool DriveTrain::moveToForward(double lpos, double rpos)
{
    static bool at_position_left = false;
    static bool at_position_right = false;

        
        if (lpos > getLeftPosition() && !at_position_left)
        {                            // if desired position is behind current encoder position
            left_motor_1->Set(-0.25); // Left motors forward 75%
        }else{
            left_motor_1->Set(0.0);
            at_position_left = true;
        }

        if (rpos > getRightPostion() && !at_position_right)
        {                             // if desired position is behind current encoder position
            right_motor_1->Set(-0.25); // Right motors forward 75%
        }else{
            right_motor_1->Set(0.0);
            at_position_right = true;
        }
        
    return at_position_left & at_position_right;
}


bool DriveTrain::moveToBackward(double lpos, double rpos){
        static bool at_position_left = false;
    static bool at_position_right = false;


        if (lpos < getLeftPosition() && !at_position_left)
        {                             // if desired position is ahead of current encoder position
            left_motor_1->Set(0.25); // Left motors reverse 75%
        }else{
            left_motor_1->Set(0.0);
            at_position_left = true;
        }
        
        if (rpos < getRightPostion() && !at_position_right)
        {                              // if desired position is ahead of current encoder position
            right_motor_1->Set(0.25); // Right motors reverse 75%
        }else{
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
    // Set the speeds for each side of the robots drive train
    left_motor_1->Set(ls);
    // leftmotor2->Set(ls);//Should be controlled by motor 1
    right_motor_1->Set(rs);
    // rightmotor2->Set(rs);//Should be controlled by motor 1
}

double DriveTrain::getLeftPosition()
{
    return -(left_encoder_1->GetPosition() + left_encoder_2->GetPosition() / 2);
}

double DriveTrain::getRightPostion()
{
    return -(right_encoder_1->GetPosition() + right_encoder_2->GetPosition() / 2);
}

double DriveTrain::getLeftPower()
{
    return -(left_motor_1->Get() + left_motor_2->Get() / 2);
}

double DriveTrain::getRightPower()
{
    return -(right_motor_1->Get() + right_motor_2->Get() / 2);
}

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