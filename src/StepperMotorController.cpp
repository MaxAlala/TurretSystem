/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   StepperMotor.cpp
 * Author: sol
 * 
 * Created on January 2, 2021, 8:44 AM
 */

#include "StepperMotorController.h"
#include "Serial_Rs232.h"
#include <iostream>
StepperMotorController::StepperMotorController(int speed1_, int speed2_, int microstepResolution_, const std::string port) :
angle2{0},
angle1{0},
steps1{0},
steps2{0},
speed1{speed1_},
speed2{speed2_},
microstepResolution{microstepResolution_}
{
    serial = Serial_Rs232::getInstance(port);
}

StepperMotorController::StepperMotorController(const StepperMotorController& orig) {

}

StepperMotorController::~StepperMotorController() {
}

void StepperMotorController::moveAFewSteps(int steps1, int steps2) {
    
    
    std::string message = "r " + std::to_string(steps1) + " " + std::to_string(speed1) + " " + std::to_string(steps2) + " " + std::to_string(speed2) + "\n";
//    std::cout << "sending this message: "<< message << "\n";
    serial->write(message);
}

void StepperMotorController::readLog() {
    
}

void StepperMotorController::moveAFewDegrees(int angle1, int angle2) {
    moveAFewSteps(angle1*stepsPer1Angle1, angle2 * stepsPer1Angle2);
}

void StepperMotorController::moveAFewSteps() {
    moveAFewSteps(steps1, steps2);
}


