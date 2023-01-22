#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

//#include <ctre/phoenixpro/TalonFX.hpp>
#include <rev/CANSparkMAX.h>
#include <frc/DigitalOutput.h>
#include "DataPacket.h"

class DriveTrain{

    public:
    DriveTrain();//Ctor
    ~DriveTrain();//Dtor
    void setSpeed(double ls, double rs);
    double* getSpeeds();//returns the current speeds of each motor
    double* getPositions();//Returns the current encoder positions for each motor encoder
    void loadTelemetry(SparkMaxPacket* dta);//Get the pointer for the telemetry data packet 
    void updateTelemetry();//Fills the telemetry struct with new data

    private:
    void configureMotors();//Used to configure motors specifically for this robot
    void setZero();//Used to set encoder positions to zero upon object creation

    rev::CANSparkMax* left_motor_1;
    rev::CANSparkMax* left_motor_2;
    rev::CANSparkMax* right_motor_1;
    rev::CANSparkMax* right_motor_2;

    rev::SparkMaxRelativeEncoder* left_encoder_1;
    rev::SparkMaxRelativeEncoder* left_encoder_2;
    rev::SparkMaxRelativeEncoder* right_encoder_1;
    rev::SparkMaxRelativeEncoder* right_encoder_2;

    SparkMaxPacket* telemetry_link;//Used to hold pointer to telemetry link

    /*TalonFX Motor Controllers
   ctre::phoenixpro::hardware::TalonFX* leftmotor1;
   ctre::phoenixpro::hardware::TalonFX* leftmotor2;
   ctre::phoenixpro::hardware::TalonFX* rightmotor1;
   ctre::phoenixpro::hardware::TalonFX* rightmotor2;
    */
};


#endif //DRIVETRAIN_H