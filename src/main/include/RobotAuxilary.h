#ifndef ROBOTAUXILARY_H
#define ROBOTAUXILARY_H

#include <frc/Doublesolenoid.h>
#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>

class RobotAuxilary
{
public:
    RobotAuxilary();
    ~RobotAuxilary();
    void calibrateArm();
    void togglePincher();
    void chram();
    void unChram();

private:
    rev::CANSparkMax *m_arm;
    rev::SparkMaxRelativeEncoder *m_arm_encoder;
    frc::Solenoid *m_chram;
    frc::DoubleSolenoid *m_pincher;
    frc::DigitalInput *m_arm_limit;
    bool is_extended;
};

#endif // ROBOTAUXILARY_H