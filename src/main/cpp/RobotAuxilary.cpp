#include "RobotAuxilary.h"
#include "Timer.h"

#include <iostream>

/**
 * @brief Construct a new Robot Auxilary:: Robot Auxilary object
 * 
 */
RobotAuxilary::RobotAuxilary()
{
    m_arm = new rev::CANSparkMax(8, rev::CANSparkMax::MotorType::kBrushless);      // Init our arm motor
    m_arm_encoder = new rev::SparkMaxRelativeEncoder(m_arm->GetEncoder());         // Get our encoder for readings
    m_chram = new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, 6);   // Init the Charlie Crammer
    m_pincher = new frc::DoubleSolenoid(frc::PneumaticsModuleType::CTREPCM, 4,5); // Init the pincher
    m_arm_limit = new frc::DigitalInput(0);                                        // Init the limit on IO 0
    m_arm->SetInverted(true);
    //m_arm->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);//Set the arm to be constantly braking when no input
    is_extended = false;//Start with chram retracted
    is_pinched = false;//Start with pincher open
    arm_position_home = 0.1;//12//Min movement for robot arm//0.0 is true home
    arm_position_max = 30.0; //22//Max movement for robot arm //30.0 is where to stop for shelf
    overtravelled_home = true;//We have not overtravelled yet
    overtravelled_max = false;//We have not overtravelled yet
}


/**
 * @brief Toggle the pincher to expand or retract
 * 
 */
void RobotAuxilary::togglePincher()
{
    if (!is_pinched)
    {
        m_pincher->Set(frc::DoubleSolenoid::kForward); // Extend/pinch pincher
        is_pinched = true;
    }
    else
    {
        m_pincher->Set(frc::DoubleSolenoid::kReverse); // Retract/unpinch pincher
        is_pinched = false;
    }
}

/**
 * @brief Extend the chram
 * 
 */
void RobotAuxilary::chram()
{
    if (!is_extended)
    {
        m_chram->Set(true); // Extend chram -- Solenoid on
        is_extended = true;
    }
}

/**
 * @brief Retract the chram
 * 
 */
void RobotAuxilary::unChram(){
    if(is_extended)
    {
        m_chram->Set(false); // Retract chram -- Solenoid off
        is_extended = false;
    }
}

//Positive speed moves arm up
//Negative speed moves arm down

/**
 * @brief Moves arm at specified speed in direction of speed
 * 
 * @param speed How fast to move arm
 * @return short 0 == not at limit, 1 == lower home limit, 2 == upper limit
 */
short RobotAuxilary::moveArm(double speed){

speed = speed/2;

std::cout<<m_arm_encoder->GetPosition()<<std::endl;
std::cout<<"LIM"<<m_arm_limit->Get()<<std::endl;


if(m_arm_limit->Get() == 0){
    m_arm_encoder->SetPosition(0.0);
}

//set min overtravel flag
if(m_arm_encoder->GetPosition() <= arm_position_home){
    overtravelled_home = true;
}

//set max overtravel flag
if(m_arm_encoder->GetPosition() >= arm_position_max){
    overtravelled_max = true;
}

//reset overtravel flags
if( m_arm_encoder->GetPosition() < arm_position_max && m_arm_encoder->GetPosition() > arm_position_home){
        overtravelled_home = false;
        overtravelled_max = false;
}
    
    if( !overtravelled_home && !overtravelled_max){
        m_arm->Set(speed);
    }else if(overtravelled_home && speed > 0.0){
        m_arm->Set(speed);
    }else if(overtravelled_max && speed < 0.0){
        m_arm->Set(speed);
    }else{//Should never get here -- sanity check
        m_arm->Set(0.0);
    }

    //Return -- slopppy
    if(overtravelled_home){
        return 1;
    }else if(overtravelled_max){
        return 2;
    }else{
        return 0;
    }
}


/**
 * @brief Destroy the Robot Auxilary:: Robot Auxilary object
 * 
 */
RobotAuxilary::~RobotAuxilary()
{
    delete m_arm;
    delete m_chram;
    delete m_pincher;
    delete m_arm_limit;
}

/*Graveyard

void RobotAuxilary::calibrateArm()
{
    static bool is_calibrated = false;

    if (!is_calibrated)
    {
        if (m_arm_limit->Get())
        {                                    // If limit is activated,
            m_arm->Set(0.0);                 // Stop motor
            m_arm_encoder->SetPosition(0.0); // Set the position of the motor to zero;
            is_calibrated = true;
        }else{
            m_arm->Set(0.5); // Move arm towards encoder;
        }
    }else{
        m_arm->Set(0.0);
    }
}


*/