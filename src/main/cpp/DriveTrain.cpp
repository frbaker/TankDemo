#include "DriveTrain.h"
#include <cmath>
//Ctor
DriveTrain::DriveTrain(){
    //Spark motor controller creation for neo brushless motors
    leftmotor1 = new rev::CANSparkMax(4,rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    leftmotor2 = new rev::CANSparkMax(5,rev::CANSparkMax::MotorType::kBrushless);
    rightmotor1 = new rev::CANSparkMax(6,rev::CANSparkMax::MotorType::kBrushless);
    rightmotor2 = new rev::CANSparkMax(7,rev::CANSparkMax::MotorType::kBrushless);
    gearshifter = new frc::DoubleSolenoid(frc::PneumaticsModuleType::CTREPCM,0,1);

    rightmotor1->SetInverted(true);
    rightmotor2->SetInverted(true);

    leftmotor2->Follow(*leftmotor1);//Have to dereference pointer since function is defined as pass by reference, so we need to
    rightmotor2->Follow(*rightmotor1);//pass the address of the object, not the address of the pointer

    //Set robot to start in low gear
    is_low_gear = true;
    /*TalonFX Motor Controllers
    leftmotor1 = new ctre::phoenixpro::hardware::TalonFX(0,"");//Leave can name empty for auto init
    leftmotor2 = new ctre::phoenixpro::hardware::TalonFX(1,"");//Leave can name empty for auto init
    rightmotor1 = new ctre::phoenixpro::hardware::TalonFX(2,"");//Leave can name empty for auto init
    rightmotor2 = new ctre::phoenixpro::hardware::TalonFX(3,"");//Leave can name empty for auto init
    */
}

void DriveTrain::setSpeed(double ls, double rs){
    //Set the speeds for each side of the robots drive train
    leftmotor1->Set(ls);
    //leftmotor2->Set(ls);//Should be controlled by motor 1
    rightmotor1->Set(rs);
    //rightmotor2->Set(rs);//Should be controlled by motor 1
}

void DriveTrain::attemptGearShift(){
        if(is_low_gear){//if we are in low gear, activate the solenoid to shift to high
        gearshifter->Set(frc::DoubleSolenoid::kForward);//SHIFT TO HIGH -- Might have to switch this depending on solenoid setup
        is_low_gear = false;//Set our gear state to whatever it is not
        }else{//if we are in high gear, only shift to low if the robot is moving slow enough
            if(std::abs(leftmotor1->Get())<=0.4 && std::abs(rightmotor1->Get())<=0.4){
                gearshifter->Set(frc::DoubleSolenoid::kReverse);//SHIFT TO LOW -- Might have to switch this depending on solenoid setup
            }
            is_low_gear = true;
        }
}

DriveTrain::~DriveTrain(){
    //Free Drive motor memory on drive train object destruction
    delete leftmotor1;
    delete leftmotor2;
    delete rightmotor1;
    delete rightmotor2;
    delete gearshifter;
}