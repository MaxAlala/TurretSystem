/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   StepperMotor.h
 * Author: sol
 *
 * Created on January 2, 2021, 8:44 AM
 */

#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include "Serial_Rs232.h"

class StepperMotorController {
public:
    StepperMotorController(int speed1_, int speed2_, int microstepResolution_, const std::string port);
    StepperMotorController(const StepperMotorController& orig);
    void readLog();
    void moveAFewSteps(int steps1, int steps2);
    void moveAFewSteps();
    void moveAFewDegrees(int angle1, int angle2);
    virtual ~StepperMotorController();
private:
    int angle2;
    int angle1;
    int steps1;
    int steps2;
    int speed1;
    int speed2;
    int stepsPer1Angle1;
    int stepsPer1Angle2;
    // 2 4 8 16 32, etc
    int microstepResolution;
    Serial_Rs232* serial;

    //    serial_stream.SetBaudRate( BaudRate::BAUD_115200 ) ;
};

#endif /* STEPPERMOTOR_H */

