#include "Autonomous.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

Autonomous::Autonomous(DriveTrain* dvtobj, RobotAuxilary* auxobj){
    drivetrain = dvtobj;
    utilties = auxobj;
    m_chooser = new frc::SendableChooser<int>;//Allocate memory for the sendable chooser
    m_chooser->SetDefaultOption("Default", 0);
    m_chooser->AddOption("Straight Forward",1);
    m_chooser->AddOption("Custom 1", 2);
    frc::SmartDashboard::PutData(m_chooser);//Put options on smart dashboard
    print_timer = new Timer(250);//Set a timer for every 250ms for debug printing
}

void Autonomous::manageAuto(){
    switch(m_chooser->GetSelected()){
        case 1:
            straightForward(310);
            break;
        case 2:
            custom1();
            break;
        default:
            defaultAuto();
            break;

    }
}

void Autonomous::defaultAuto(){
    static int steps = 0;
    
    if(print_timer->getTimer()){
        std::cout<<"Left Position: "<<drivetrain->getLeftPosition()<<std::endl;
        std::cout<<"Right Position: "<<drivetrain->getRightPostion()<<std::endl;
        std::cout<<"Angle: "<<drivetrain->getAngle()<<std::endl;
    }

    switch(steps){
        case 0:
            if(drivetrain->absoluteMoveForward(310,310)){
                steps++;
            }
            break;
        case 1:
            if(drivetrain->absoluteMoveBackward(270,270)){
                steps++;
            }
            break;
        default:
            break;
    }

}

void Autonomous::straightForward(double dist){
    if(print_timer->getTimer()){
        std::cout<<"Left Position: "<<drivetrain->getLeftPosition()<<std::endl;
        std::cout<<"Right Position: "<<drivetrain->getRightPostion()<<std::endl;
        std::cout<<"Angle: "<<drivetrain->getAngle()<<std::endl;
    }
    drivetrain->absoluteMoveForward(dist,dist);
}

void Autonomous::custom1(){
    static int steps = 0;
    switch(steps){
        
        case 0:
            if(drivetrain->relativeMoveForward(50,50)){
                utilties->chram();
                steps++;
                drivetrain->resetFlags();//Reset the flags to enable further steps
            }
            break;
        case 1:
            if(drivetrain->relativeMoveBackward(50,50)){
                utilties->unChram();
                steps++;
                drivetrain->resetFlags();//Reset the flags to enable further steps
            }
            break;
        case 2:
            if(drivetrain->absoluteTurnCW(90)){
                steps++;
                drivetrain->resetFlags();//Reset the flags to enable further steps
            }
            break;
        case 3:
            if(drivetrain->relativeMoveForward(50,50)){
                steps++;
                drivetrain->resetFlags();//Reset the flags to enable further steps
            }
            break;
        case 4:
            if(drivetrain->relativeMoveBackward(50,50)){
                steps++;
                drivetrain->resetFlags();//Reset the flags to enable further steps
            }
            break;
        case 5:
            if(drivetrain->absoluteTurnCW(180)){
                steps++;
                drivetrain->resetFlags();//Reset the flags to enable further steps
            }
            break;
        case 6:
            if(drivetrain->relativeMoveForward(50,50)){
                steps++;
                drivetrain->resetFlags();//Reset the flags to enable further steps
            }
            break;
        case 7:
            if(drivetrain->relativeMoveBackward(50,50)){
                steps++;
                drivetrain->resetFlags();//Reset the flags to enable further steps
            }
            break;
        case 8:
            if(drivetrain->absoluteTurnCW(270)){
                steps++;
                drivetrain->resetFlags();//Reset the flags to enable further steps
            }
            break;
        case 9:
            if(drivetrain->relativeMoveForward(50,50)){
                steps++;
                drivetrain->resetFlags();//Reset the flags to enable further steps
            }
            break;
        case 10:
            if(drivetrain->relativeMoveBackward(50,50)){
                steps++;
                drivetrain->resetFlags();//Reset the flags to enable further steps
            }
            break;
        case 11:
            if(drivetrain->absoluteTurnCW(359)){
                steps++;
                drivetrain->resetFlags();//Reset the flags to enable further steps
            }
            break;
        
        default:
            break;
    }
}




Autonomous::~Autonomous(){
    delete m_chooser;
}