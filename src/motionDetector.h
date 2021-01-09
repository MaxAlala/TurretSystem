/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MotionDetector.h
 * Author: sol
 *
 * Created on January 1, 2021, 2:35 PM
 */

#ifndef MOTIONDETECTOR_H
#define MOTIONDETECTOR_H
#include <vector>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include <vector>
#include <stdio.h>
#include <iostream>
#include <opencv2/tracking.hpp>
//#include <opencv2 core="" ocl.hpp="">
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
using namespace std;
using namespace cv;

class MotionDetector {
public:

    virtual ~MotionDetector();
    // cant use implicit + explicit 
    MotionDetector(const MotionDetector& orig) = delete;
    void operator=(const MotionDetector&) = delete;
    MotionDetector* getInstance(
            int x_resolution,
            int y_resolution,
            int x_divider,
            int y_divider,
            bool shouldFlipCam,
            int rectWidth,
            int rectHeight
            );
    void findMotion(VideoCapture& cap, Point& detectedObject, int framesToSkip);
    
    
    
private:
    int currentGroupNumber;
    int x_divider;
    int y_divider;
    //calculate area motion weight of 100 areas == chose the best one
    vector<int> motionAreaWeights;
    int x_resolution;
    int y_resolution;
    int x_pixels_in_cell = x_resolution / x_divider;
    int y_pixels_in_cell = y_resolution / y_divider;
    vector<vector<int>> weightedCells;
    vector<vector<int>> groupedCells;
    vector<Point> groupAverage;
    vector<Point> topLeft;
    vector<Point> bottomRight;
    int shouldFlipCam;
    int rectWidth;
    int rectHeight;
    MotionDetector(
            int x_resolution,
            int y_resolution,
            int x_divider,
            int y_divider,
            bool shouldFlipCam,
            int rectWidth,
            int rectHeight
            );
    void checkNeighbourhood(int i, int j);
    void printGroupCellMatrix();
    static MotionDetector * p_motionDetector;
    vector<int > groupCount;
};

#endif /* MOTIONDETECTOR_H */

