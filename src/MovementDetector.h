/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MovementDetector.h
 * Author: sol
 *
 * Created on January 14, 2021, 7:09 PM
 */

#ifndef MOVEMENTDETECTOR_H
#define MOVEMENTDETECTOR_H
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
class MovementDetector {
public:
    MovementDetector();
    MovementDetector(const MovementDetector& orig);
    void findMovement(cv::Mat& img, cv::Point& detectedObject);
    void checkNeighbourhood(int i, int j);

    virtual ~MovementDetector();
private:

};

#endif /* MOVEMENTDETECTOR_H */

