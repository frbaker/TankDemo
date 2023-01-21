#include "DriveTrain.h"
//Ctor
DriveTrain::DriveTrain(){
    //Spark motor controller creation for neo brushless motors
    leftmotor1 = new rev::CANSparkMax(4,rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    leftmotor2 = new rev::CANSparkMax(5,rev::CANSparkMax::MotorType::kBrushless);
    rightmotor1 = new rev::CANSparkMax(6,rev::CANSparkMax::MotorType::kBrushless);
    rightmotor2 = new rev::CANSparkMax(7,rev::CANSparkMax::MotorType::kBrushless);

    leftmotor1->SetInverted(true);//Invert left side motors since they are flipped
    leftmotor2->SetInverted(true);//Invert left side motors since they are flipped

    leftmotor2->Follow(*leftmotor1);//Have to dereference pointer since function is defined as pass by reference, so we need to
    rightmotor2->Follow(*rightmotor1);//pass the address of the object, not the address of the pointer

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



DriveTrain::~DriveTrain(){
    //Free Drive motor memory on drive train object destruction
    delete leftmotor1;
    delete leftmotor2;
    delete rightmotor1;
    delete rightmotor2;
}