#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

//#include <ctre/phoenixpro/TalonFX.hpp>
#include <rev/CANSparkMAX.h>
#include <frc/DigitalOutput.h>

class DriveTrain{

    public:
    DriveTrain();//Ctor
    ~DriveTrain();//Dtor
    void setSpeed(double ls, double rs);
    void getSpeeds();//returns the current speeds of each motor
    
    private:
    rev::CANSparkMax* left_motor_1;
    rev::CANSparkMax* left_motor_2;
    rev::CANSparkMax* right_motor_1;
    rev::CANSparkMax* right_motor_2;

    rev::SparkMaxRelativeEncoder* left_encoder_1;
    rev::SparkMaxRelativeEncoder* left_encoder_2;
    rev::SparkMaxRelativeEncoder* right_encoder_1;
    rev::SparkMaxRelativeEncoder* right_encoder_2;


    /*TalonFX Motor Controllers
   ctre::phoenixpro::hardware::TalonFX* leftmotor1;
   ctre::phoenixpro::hardware::TalonFX* leftmotor2;
   ctre::phoenixpro::hardware::TalonFX* rightmotor1;
   ctre::phoenixpro::hardware::TalonFX* rightmotor2;
    */
};


#endif //DRIVETRAIN_H