/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Eye.h
 * Author: sol
 *
 * Created on January 2, 2021, 11:46 AM
 */

#ifndef EYE_H
#define EYE_H
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp" // circle

class Eye {
public:
    Eye(int camera_id, bool shouldFlipFrame = false);
    Eye(const Eye& orig);
    virtual ~Eye();
    cv::Point run();

private:
    bool shouldFlipFrame;
    cv::Point chosenPoint;
    bool wasPointChosen;
    cv::VideoCapture cap;
    cv::Mat frame;
    const std::string selectionWindow;
    const std::string selectedPointWindow;
};

#endif /* EYE_H */

