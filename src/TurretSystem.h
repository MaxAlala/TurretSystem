/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TurretSystem.h
 * Author: sol
 *
 * Created on January 2, 2021, 8:43 AM
 */

#ifndef TURRETSYSTEM_H
#define TURRETSYSTEM_H

#include "PixelToMotorStepsConverter.h"
#include "StepperMotorController.h"
#include <memory> // для std::unique_ptr
#include "Eye.h"
#include "MovementDetector.h"
//turn on
//turn off
//manual state
//automatic state ==
//  direct the turret + shoot
//  direct the turret + wait the command

class TurretSystem {

    enum class States {
        AUTOMATIC,
        MANUAL,
    };

public:
    TurretSystem();
    TurretSystem(const TurretSystem& orig);
    bool turnOff();
    bool turnOn();
    bool setState(States state);
    void run();
    virtual ~TurretSystem();
    void calibrateItself();
private:

    States currentState;
    std::unique_ptr<Eye> eye;
    std::unique_ptr<StepperMotorController> stepperMotorController;
    std::unique_ptr<PixelToMotorStepsConverter> pixelToMotorStepsConverter;

};

#endif /* TURRETSYSTEM_H */
