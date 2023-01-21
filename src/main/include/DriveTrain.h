#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

//#include <ctre/phoenixpro/TalonFX.hpp>
#include <rev/CANSparkMAX.h>
#include <frc/DigitalOutput.h>
#include "Encoders.h"

class DriveTrain{

    public:


    DriveTrain();//Ctor
    ~DriveTrain();//Dtor
    void setSpeed(double ls, double rs);
    struct SparkMaxEncoderBundle* getEncoders();
    private:

    void initEncoders();//Used in constructor to populate the encoder struct
    void deleteEncoders();//Used in destructor to free encoder struct

    rev::CANSparkMax* leftmotor1;
    rev::CANSparkMax* leftmotor2;
    rev::CANSparkMax* rightmotor1;
    rev::CANSparkMax* rightmotor2;

    struct SparkMaxEncoderBundle* drive_base_encoders;


    /*TalonFX Motor Controllers
   ctre::phoenixpro::hardware::TalonFX* leftmotor1;
   ctre::phoenixpro::hardware::TalonFX* leftmotor2;
   ctre::phoenixpro::hardware::TalonFX* rightmotor1;
   ctre::phoenixpro::hardware::TalonFX* rightmotor2;
    */
};


#endif //DRIVETRAIN_H