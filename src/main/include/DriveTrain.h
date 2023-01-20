#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

//#include <ctre/phoenixpro/TalonFX.hpp>
#include <rev/CANSparkMAX.h>
#include "frc/DoubleSolenoid.h"
#include "frc/DigitalOutput.h"

class DriveTrain{

    public:
    DriveTrain();//Ctor
    ~DriveTrain();//Dtor
    void setSpeed(double ls, double rs);
    void attemptGearShift();
    private:

    rev::CANSparkMax* leftmotor1;
    rev::CANSparkMax* leftmotor2;
    rev::CANSparkMax* rightmotor1;
    rev::CANSparkMax* rightmotor2;

    frc::DoubleSolenoid* gearshifter;

    bool is_low_gear;

    /*TalonFX Motor Controllers
   ctre::phoenixpro::hardware::TalonFX* leftmotor1;
   ctre::phoenixpro::hardware::TalonFX* leftmotor2;
   ctre::phoenixpro::hardware::TalonFX* rightmotor1;
   ctre::phoenixpro::hardware::TalonFX* rightmotor2;
    */
};


#endif //DRIVETRAIN_H