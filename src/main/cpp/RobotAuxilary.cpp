#include "RobotAuxilary.h"
#include "Timer.h"


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
    is_extended = false;//Start with chram retracted
}

/**
 * @brief Called continuously during autonomous periodic, but only executes until calibrated
 * 
 */
void RobotAuxilary::calibrateArm()
{
    static bool is_calibrated = false;

    if (!is_calibrated)
    {
        m_arm->Set(0.5); // Move arm towards encoder;
        if (m_arm_limit->Get())
        {                                    // If limit is activated,
            m_arm->Set(0.0);                 // Stop motor
            m_arm_encoder->SetPosition(0.0); // Set the position of the motor to zero;
            is_calibrated = true;
        }
    }else{
        m_arm->Set(0.0);
    }
}

/**
 * @brief Toggle the pincher to expand or retract
 * 
 */
void RobotAuxilary::togglePincher()
{
    static bool is_pinched = false; // Set initial pinch to false

    if (is_pinched)
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