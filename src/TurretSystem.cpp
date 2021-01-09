/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TurretSystem.cpp
 * Author: sol
 * 
 * Created on January 2, 2021, 8:43 AM
 */

#include "TurretSystem.h"
#include "StepperMotorController.h"
#include <iostream>
TurretSystem::TurretSystem() : currentState{States::MANUAL}
{
    eye.reset(new Eye(1, true));
    pixelToMotorStepsConverter.reset(new PixelToMotorStepsConverter(640, 480, 561, 22, 12, 2.905, 1.57));
    stepperMotorController.reset(new StepperMotorController(400, 400, 4, "/dev/ttyUSB0"));

}

TurretSystem::TurretSystem(const TurretSystem& orig) {

}

TurretSystem::~TurretSystem() {

}

bool TurretSystem::turnOff() {

}

bool TurretSystem::turnOn() {

}

bool TurretSystem::setState(States state) {
    currentState = state;
}

void TurretSystem::run() {
    for (;;) {
        if (currentState == States::MANUAL) {
            cv::Point chosenPoint = eye->run();
            if (chosenPoint == cv::Point(0, 0)) {

            } else {
                std::cout << "TurretSystem::run().received point: " << chosenPoint.x << " " << chosenPoint.y << std::endl;
                int stepsForFirstMotor = 0;
                int stepsForSecondMotor = 0;
//                pixelToMotorStepsConverter->calculateStepsByPixel(chosenPoint, stepsForFirstMotor, stepsForSecondMotor);
//                pixelToMotorStepsConverter->calculateStepsUsingCalibratedValue(chosenPoint, stepsForFirstMotor, stepsForSecondMotor);
                pixelToMotorStepsConverter->calculateStepsUsingInverseKinematics(chosenPoint, stepsForFirstMotor, stepsForSecondMotor);

                stepperMotorController->moveAFewSteps(stepsForFirstMotor, stepsForSecondMotor);

            }
            if (cv::waitKey(5) >= 0) // waits any button to exit the program
                break;
        } else {
        }
    }
}

// idea ==
//1 manually point to some specific point. Remember it.
//2 send robot command to perform 250 and 250 steps
//3 find your specific point + choose it to calculate x and y pixel distance
//4 now u know how much costs one pixel in steps

void TurretSystem::calibrateItself() {
    //2
    int moveStepsNumber = 150;
    stepperMotorController->moveAFewSteps(moveStepsNumber, moveStepsNumber);
    cv::Point chosenPoint;
    //3 an initial pixel
    for (;;) {
        chosenPoint = eye->run();
        if (chosenPoint != cv::Point(0, 0)) {
            break;
        }
        if (cv::waitKey(5) >= 0) // waits any button to exit the program
            break;
    }
    int xPixelNumber = abs(320 - chosenPoint.x);
    int yPixelNumber = abs(240 - chosenPoint.y);
    std::cout << "x and y pixel number:" << xPixelNumber << " " << yPixelNumber << std::endl;

    double numberOfStepsInOnePixelX = moveStepsNumber / xPixelNumber;
    double numberOfStepsInOnePixelY = moveStepsNumber / yPixelNumber;
    std::cout << "x and y pixels in steps:" << numberOfStepsInOnePixelX << " " << numberOfStepsInOnePixelY << std::endl;
    //    //4

}

