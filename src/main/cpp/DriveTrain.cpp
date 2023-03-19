#include "DriveTrain.h"

#include <cmath>

double L = 27.25; //axle to axle measurement from front to back?
double W = 22.125; //axle to axle measurement from left to right?

/**
 * @brief Construct a new Drive Train:: Drive Train object
 *
 */
DriveTrain::DriveTrain()
{
    front_left_drive = new TalonSRX(1);
    front_left_swerve = new TalonSRX(2);
    front_right_drive = new TalonSRX(3);
    front_right_swerve = new TalonSRX(4);

    back_left_drive = new TalonSRX(5);
    back_left_swerve = new TalonSRX(6);
    back_right_drive = new TalonSRX(7);
    back_right_swerve = new TalonSRX(8);

    front_left_encoder = new frc::AnalogEncoder(0);
    front_right_encoder =  new frc::AnalogEncoder(1);
    back_left_encoder =  new frc::AnalogEncoder(2);
    back_right_encoder =  new frc::AnalogEncoder(3);

    // Gyro
    gyro = new PigeonIMU(16); // Create our gyro object with can ID 16
                                                      // Zero all sensors
    setZero();
    resetFlags(); // Reset movement flags
}

/**
 * @brief Oh boy, This takes 3 joystick inputs and converts it to
 * swervedrive relative values that can be passed to the swerve modules 
 * 4 doubles for speed and 4 doubles for direction 
 * x1 : Joystick1 X axis
 * y1 : Joystick1 Y axis
 * x2 : Joystick2 X axis
 *
 */
void DriveTrain::Drive(double x1, double y1, double x2){
    double r = sqrt((L*L) + (W*W));
    y1 *= -1;
    double a = x1 - x2 * (L/r);
    double b = x1 +x2 * (L/r);
    double c = y1 - x2 * (W/r);
    double d = y1 + x2 * (W/r);

    double backRightSpeed = sqrt ((a * a) + (d * d));
    double backLeftSpeed = sqrt ((a * a) + (c * c));
    double frontRightSpeed = sqrt ((b * b) + (d * d));
    double frontLeftSpeed = sqrt ((b * b) + (c * c));

    double backRightAngle = atan2 (a, d) / 3.141592653;
    double backLeftAngle = atan2 (a, c) / 3.141592653;
    double frontRightAngle = atan2 (b, d) / 3.141592653;
    double frontLeftAngle = atan2 (b, c) / 3.141592653;
}

/**
 * @brief WheelDrive represents an individual swerve module
 * Takes the canId for the angleMotor, canId for the speedMotor
 * and analog port of the encoder of the swerve module
 */
void DriveTrain::WheelDrive (int angleMotor, int speedMotor, int encoder) {
    this->angleMotor = new TalonSRX(angleMotor);
    this->speedMotor = new TalonSRX(speedMotor);
    this->imu = new frc::AnalogEncoder(encoder);

    pidController = new frc2::PIDController(1, 0, 0);

   //todo- not sure what to use for these values
    pidController->EnableContinuousInput(0.0,360.0);

    //todo - not sure what controlMode to use, not sure what setpoint to use
    this->angleMotor->Set(ControlMode::PercentOutput,pidController->Calculate(this->imu->GetDistance(), 100));
    this->speedMotor->Set(ControlMode::PercentOutput,pidController->Calculate(this->speedMotor->GetMotorOutputPercent(), .5));

}


/**
 * @brief Configure the motors to turn in the correct direction relative to robot forward.
 * Used at startup
 *
 */
void DriveTrain::configureMotors()
{
    // Robot specific motor settings
    front_left_drive->SetInverted(true); // Invert left side motors since they are flipped
    front_left_swerve->SetInverted(true); // Invert left side motors since they are flipped
    //
    front_left_swerve->Follow(*front_left_drive);   // Have to dereference pointer since function is defined as pass by reference, so we need to
    front_right_swerve->Follow(*front_right_drive); // pass the address of the object, not the address of the pointer
    setCoastMode();//Set our robot to start in coast mode
}

/**
 * @brief Set all encoders to zero. Used at startup
 *
 */
void DriveTrain::setZero()
{
    // Set the start position of the motor encoders to zero
  
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
        front_left_drive->Set(ControlMode::PercentOutput,-0.75); // Left motors forward 75%
        

    }
    else
    {
        front_left_drive->Set(ControlMode::PercentOutput,0.0);
        at_position_left = true;
    }

    if (rpos + relative_right_pos_zero > getRightPostion() && !at_position_right)
    {                              // if desired position is behind current encoder position
        front_right_drive->Set(ControlMode::PercentOutput,-0.75); // Right motors forward 75%
    }
    else
    {
        front_right_drive->Set(ControlMode::PercentOutput,0.0);
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
        front_left_drive->Set(ControlMode::PercentOutput,0.75); // Left motors reverse 75%
    }
    else
    {
        front_left_drive->Set(ControlMode::PercentOutput,0.0);
        at_position_left = true;
    }

    if (relative_right_pos_zero - rpos < getRightPostion() && !at_position_right)
    {                             // if desired position is ahead of current encoder position
        front_right_drive->Set(ControlMode::PercentOutput,0.75); // Right motors reverse 75%
    }
    else
    {
        front_right_drive->Set(ControlMode::PercentOutput,0.0);
        at_position_right = true;
    }

    return at_position_left & at_position_right;
}


void DriveTrain::setCoastMode(){
    front_left_drive->SetNeutralMode(NeutralMode::Coast);
    front_left_swerve->SetNeutralMode(NeutralMode::Coast);
    front_right_drive->SetNeutralMode(NeutralMode::Coast);
    front_right_swerve->SetNeutralMode(NeutralMode::Coast);
    /*front_left_drive->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    front_right_drive->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    */
}


void DriveTrain::setBreakMode(){

    front_left_drive->SetNeutralMode(NeutralMode::Brake);
    front_left_swerve->SetNeutralMode(NeutralMode::Brake);
    front_right_drive->SetNeutralMode(NeutralMode::Brake);
    front_right_swerve->SetNeutralMode(NeutralMode::Brake);
    /*
    front_left_drive->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    front_right_drive->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    */
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
    front_left_drive->Set(ControlMode::PercentOutput,ls);
    // leftmotor2->Set(ls);//Should be controlled by motor 1
    front_right_drive->Set(ControlMode::PercentOutput,rs);
    // rightmotor2->Set(rs);//Should be controlled by motor 1
}

/**ControlMode::PercentOutput,
 * @brief Returns the current encoder reading of the left side motors
 *
 * @return double Left encoder position
 */
double DriveTrain::getLeftPosition()
{
    return -(front_left_encoder->GetAbsolutePosition() + front_right_encoder->GetAbsolutePosition() / 2);
}

/**
 * @brief Returns the current encoder reading of the right side motors
 *
 * @return double Right encoder position
 */
double DriveTrain::getRightPostion()
{
    return -(back_left_encoder->GetAbsolutePosition() + back_right_encoder->GetAbsolutePosition() / 2);
}

/**
 * @brief Returns the current power outpout of the left side motors
 *
 * @return double Power output
 */
double DriveTrain::getLeftPower()
{

    return -(front_left_drive->GetOutputCurrent() + front_left_swerve->GetOutputCurrent() / 2);
}

/**
 * @brief Returns the current power output of the right side motors
 *
 * @return double Power output
 */
double DriveTrain::getRightPower()
{
    return -(front_right_drive->GetOutputCurrent() + front_right_swerve->GetOutputCurrent() / 2);
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
 /*   // Motors
    delete front_left_drive;
    delete front_left_swerve;
    delete front_right_drive;
    delete front_right_swerve;
    // Encoders
    delete front_left_encoder;
    delete front_right_encoder;
    delete back_left_encoder;
    delete back_right_encoder;
    // Gyro
    */
   delete front_left_drive;
    delete front_left_swerve;
    delete front_right_drive;
    delete front_right_swerve;

    delete back_left_drive;
    delete back_left_swerve;
    delete back_right_drive;
    delete back_right_swerve;



    // Encoders
    delete front_left_encoder;
    delete front_right_encoder;
    delete back_left_encoder;
    delete back_right_encoder;
    delete gyro;
}

/*Graveyard

bool DriveTrain::absoluteMoveForward(double lpos, double rpos)
{
    if (lpos > getLeftPosition() && !at_position_left)
    {                             // if desired position is behind current encoder position
        front_left_drive->Set(-0.25); // Left motors forward 75%
    }
    else
    {
        front_left_drive->Set(0.0);
        at_position_left = true;
    }

    if (rpos > getRightPostion() && !at_position_right)
    {                              // if desired position is behind current encoder position
        front_right_drive->Set(-0.25); // Right motors forward 75%
    }
    else
    {
        front_right_drive->Set(0.0);
        at_position_right = true;
    }

    return at_position_left & at_position_right;
}

bool DriveTrain::absoluteMoveBackward(double lpos, double rpos)
{
    if (lpos < getLeftPosition() && !at_position_left)
    {                            // if desired position is ahead of current encoder position
        front_left_drive->Set(0.25); // Left motors reverse 75%
    }
    else
    {
        front_left_drive->Set(0.0);
        at_position_left = true;
    }

    if (rpos < getRightPostion() && !at_position_right)
    {                             // if desired position is ahead of current encoder position
        front_right_drive->Set(0.25); // Right motors reverse 75%
    }
    else
    {
        front_right_drive->Set(0.0);
        at_position_right = true;
    }

    return at_position_left & at_position_right;
}

*/