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
    short moveArm(double speed);

private:
    rev::CANSparkMax *m_arm;
    rev::SparkMaxRelativeEncoder *m_arm_encoder;
    frc::Solenoid *m_chram;
    frc::DoubleSolenoid *m_pincher;
    frc::DigitalInput *m_arm_limit;
    bool is_pinched;
    bool is_extended;
    double arm_position_max;
    double arm_position_home;
    bool overtravelled_home, overtravelled_max;
};

#endif // ROBOTAUXILARY_H