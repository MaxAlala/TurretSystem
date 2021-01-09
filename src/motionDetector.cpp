/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MotionDetector.cpp
 * Author: sol
 * 
 * Created on January 1, 2021, 2:35 PM
 */

#include "motionDetector.h"
#include <cstddef>
#include <iostream>

using namespace std;




MotionDetector* MotionDetector::p_motionDetector{nullptr};

MotionDetector::MotionDetector(
        int x_resolution,
        int y_resolution,
        int x_divider,
        int y_divider,
        bool shouldFlipCam,
        int rectWidth,
        int rectHeight
        ) :
x_resolution{x_resolution},
y_resolution{y_resolution},
x_divider{x_divider},
y_divider{y_divider},
shouldFlipCam{shouldFlipCam},
rectWidth{rectWidth},
rectHeight{rectHeight},
groupAverage{vector<Point >(10)},
x_pixels_in_cell{x_resolution / x_divider},
y_pixels_in_cell{y_resolution / y_divider},
topLeft{vector<Point>(10)},
bottomRight{vector<Point> (10)},
weightedCells{vector<int>(x_divider)},
motionAreaWeights{vector<int>(x_divider * y_divider, 0)},
currentGroupNumber{1},
groupCount{vector<int>(10)}
{

    cout << "inside the constructor.";
    //calculate area motion weight of 100 areas == chose the best one
    for (int i = 0; i < x_divider; i++) {
        weightedCells.at(i) = vector<int>(y_divider);
    }
}

MotionDetector::~MotionDetector() {
}

MotionDetector* MotionDetector::getInstance(
        int x_resolution,
        int y_resolution,
        int x_divider,
        int y_divider,
        bool shouldFlipCam,
        int rectWidth,
        int rectHeight) {
    if (p_motionDetector == nullptr) {
        p_motionDetector == new MotionDetector(
                x_resolution,
                y_resolution,
                x_divider,
                y_divider,
                shouldFlipCam,
                rectWidth,
                rectHeight);
    }
    return p_motionDetector;
}

// checks if a cell is within the bounds + there was a movement in a cell
// + group cell is not busy
// infills groupedCells + groupAverage + groupCount  using currentGroupNumber 

void MotionDetector::checkNeighbourhood(int i, int j) {

    if (
            (i >= 0 && j >= 0 && i < x_divider && j < y_divider)
            && (weightedCells[i][j] != 0)
            && (groupedCells[i][j] == 0)

            ) {

        // set group
        groupedCells[i][j] = currentGroupNumber;

        // change an average cell == where should the turret point
        groupAverage[currentGroupNumber].x += i;
        groupAverage[currentGroupNumber].y += j;

        // counter count cells of each group
        groupCount[currentGroupNumber]++;

        // check all neighborhood except top left one
        // x->
        //y
        //|
        //\/
        checkNeighbourhood(i, j - 1);
        checkNeighbourhood(i + 1, j - 1);
        checkNeighbourhood(i + 1, j);
        checkNeighbourhood(i + 1, j + 1);
        checkNeighbourhood(i, j + 1);
        checkNeighbourhood(i - 1, j + 1);
        checkNeighbourhood(i - 1, j);
    }
}

void MotionDetector::printGroupCellMatrix() {
    cout << "cell group matrix: \n";
    for (int i = 0; i < x_divider; i++) {
        for (int j = 0; j < y_divider; j++) {
            cout << groupedCells[i][j] << " ";
        }
        cout << "\n";
    }


    cout << "cell weight matrix: \n";
    for (int i = 0; i < x_divider; i++) {
        for (int j = 0; j < y_divider; j++) {
            cout << weightedCells[i][j] << " ";
        }
        cout << "\n";
    }
}

void MotionDetector::findMotion(VideoCapture& cap, Point& detectedObject, int framesToSkip) {
    Mat flow;
    cout << "FINDING MOTION!!! \n";

    // some faster than mat image container
    UMat flowUmat, prevgray;
    int minLengthSizeOfSubtractionVector = 1500;

    //    Point topLeftAreaPoint;
    //    Point bottomRightAreaPoint;
    int movingObjectWasDetected = 0;

    ///////////////////////////// SERIAL PORT CONNECTION
    using namespace LibSerial;
    using LibSerial::SerialPort;
    using LibSerial::SerialStream;
    SerialPort serial_port;
    SerialStream serial_stream;
    serial_port.Open("/dev/ttyUSB0");
    //    serial_stream.Open( "/dev/ttyUSB1" ) ;

    // Specify a timeout value (in milliseconds).
    size_t timeout_milliseconds = 25;

    // Set the baud rates.
    using LibSerial::BaudRate;
    serial_port.SetBaudRate(BaudRate::BAUD_115200);
    //    serial_stream.SetBaudRate( BaudRate::BAUD_115200 ) ;

    int steps = 10;
    int stepsForMotionSlowing = 50;
    int speeds [] = {200, 200};
    int bigSpeed = 1600;
    ///////////////////////////// END OF SERIAL PORT DECLARATION
    namedWindow("prew", WINDOW_AUTOSIZE);
    Mat original;
    for (;;) {
        //read == grab(takes frame from camera) + retrieve(decode the taken frame)
        bool wasFrameTaken = cap.grab();
        if (wasFrameTaken == false) {

            // if video capture failed
            cout << "Video Capture Fail" << endl;
            break;
        } else {
            Mat img;

            // capture frame from video file
            cap.retrieve(img, CV_CAP_OPENNI_BGR_IMAGE);
            if (shouldFlipCam)flip(img, img, 1);
            resize(img, img, Size(640, 480));

            // save original for later
            img.copyTo(original);
            resize(original, original, Size(640, 480));

            // just make current frame gray
            cvtColor(img, img, COLOR_BGR2GRAY);

            // For all optical flow you need a sequence of images.. Or at least 2 of them. Previous 
            // and current frame
            // if there is no current frame
            // go to this part and fill previous frame
            // else {
            // img.copyTo(prevgray);
            //   }
            // if previous frame is not empty.. There is a picture of previous frame. Do some
            // optical flow alg. 

            if (prevgray.empty() == false) {

                // calculate optical flow 
                calcOpticalFlowFarneback(prevgray, img, flowUmat, 0.4, 1, 12, 2, 8, 1.2, 0);

                // copy Umat container to standard Mat
                flowUmat.copyTo(flow);
                int y_step = 5;
                int x_step = 5;
                // By y += 5, x += 5 you can specify the grid 
                for (int y = 0; y < original.rows; y += y_step) {
                    for (int x = 0; x < original.cols; x += x_step) {

                        // 106x6 160x3
                        // get the flow from y, x position * 10 for better visibility
                        int vector_multiplier = 10;
                        const Point2f flowatxy = flow.at<Point2f>(y, x) * vector_multiplier;

                        // draw line at flow direction, first point is pixel coordinate, second pixels is a flow direction coordinate
                        line(original, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255, 0, 0));

                        // draw initial point
                        circle(original, Point(x, y), 1, Scalar(0, 0, 0), -1);
                    }
                }
                bool needToCheckMotionArea = 1;
                bool wasMovementDetected = false;
                if (needToCheckMotionArea) {

                    // this part infill weightedCells
                    //divides a frame into parts
                    for (int i = 0; i < x_divider; i++) {
                        for (int j = 0; j < y_divider; j++) {
                            //goes through all points of a current cell
                            for (int x = x_pixels_in_cell * i; x < x_pixels_in_cell * (i + 1); x++) {
                                for (int y = y_pixels_in_cell * j; y < y_pixels_in_cell * (j + 1); y++) {

                                    //returns a point describing orientation and speed
                                    const Point2f flowatxy = flow.at<Point2f>(x, y) * 100;

                                    // creates parameters describing size of a motion
                                    Point difference(x - (x + flowatxy.x), y - (y + flowatxy.y));
                                    int length = sqrt(difference.x * difference.x + difference.y * difference.y);
                                    if (length > minLengthSizeOfSubtractionVector) weightedCells[i][j] += length;
                                }
                            }
                        }
                    }
                    // creates motion groups
                    for (int i = 0; i < x_divider; i++)
                        for (int j = 0; j < y_divider; j++) {
                            // if a cell with motion + without a group == creates a new group
                            if ((weightedCells[i][j] != 0) && (groupedCells[i][j] == 0)) {
                                wasMovementDetected = true;
                                
                                // adds all neighbors to a group 
                                checkNeighbourhood(i, j);
                                
                                // then finished adding all neighbors to a group, changes a group
                                ++currentGroupNumber;
                            }
                        }
                }

                int maxGroup = 0;
                if (wasMovementDetected) {
                    for (int i = 1; i < groupCount.size(); i++) {
                        if (groupCount[maxGroup] < groupCount[i]) {
                            maxGroup = i;
                        }
                    }

                    
                    printGroupCellMatrix();
                    cout << "groups count: \n";
                    for (int i = 0; i < groupCount.size(); i++) {
                        cout << groupCount.at(i) << " ";
                    }
                    cout << "\n";

                    cout << "MAX group is " << maxGroup << endl;
                    detectedObject = Point(groupAverage[maxGroup].x * x_pixels_in_cell / groupCount[maxGroup], groupAverage[maxGroup].y * y_pixels_in_cell / groupCount[maxGroup]);

                    cout << "detected Object: " << detectedObject.x << " " << detectedObject.y << endl;

///////////////////////////
//                    calculateAngles(detectedObject);
//                    //////////////////////////// SENDING PART
//                    string str;
//                    string message = "r 0 0 0 0 0 0 " + to_string(angleY * stepsPer1Angle[3]) + " " + to_string(axisSpeeds[3]) + " " + to_string(angleX * stepsPer1Angle[4]) + " " + to_string(axisSpeeds[4]) + "\n";
///////end
                    //                    char modelWasChoosen = ' ';
                    //                    while (modelWasChoosen != 'Y' && modelWasChoosen != 'N') {
                    //                        std::cout << "Error invalid input please try again " << std::endl;
                    //                        std::cout << "Please enter [Y/N] if u or not to send detected motion: ";
                    //                        std::cin >> modelWasChoosen;
                    //                        modelWasChoosen = toupper(modelWasChoosen);
                    //                    }



                    //////////display image

                    circle(original, Point(detectedObject.y, detectedObject.x), 30, Scalar(0, 0, 255), 5);
                    //                rectangle(original, rect, Scalar(255, 0, 0), 2, 1);
                    // draw the results
                    //                    namedWindow("prew", WINDOW_AUTOSIZE);

                    imshow("prewwwww", original);
                    
                    //////////////////////2
//                    int key1 = waitKey(21);
//                    ////////// end of displaying
//                    //                    std::cout << "u have pressed: " << modelWasChoosen;
//                    //                    if (modelWasChoosen == 'Y') {
//                    serial_port.Write(message); // go up
//                    //                      serial_port.Write("mca 4 200 0\n"); // go up
//                    cout << "THIS MESSAGE WAS SENT: " << message << "\n";
//                    ;
//                    while (str.substr(0, 7) != "running") {
//                        str = "";
//                        if (serial_port.GetNumberOfBytesAvailable() != 0) serial_port.ReadLine(str);
//                        if (str != "") cout << str << "\n";
//                        //                        cout << str.substr(0, 7) << endl;
//                    }
//                    //                         str = "";
//                    cout << "sending was finished. \n";
//                    //                    }
//                    //////////////////////// END OF SENDING PART
//
//                    setDefaultGroupCellMatrix();
//                    // make prevgray empty to get else section
//                    prevgray = UMat();
//                    framesToSkip = 5;
//                    
                    
                    /////////////////////////////end
                } else {
                    //////////display image

                    circle(original, Point(detectedObject.y, detectedObject.x), 30, Scalar(0, 0, 255), 5);
                    //                rectangle(original, rect, Scalar(255, 0, 0), 2, 1);
                    // draw the results
                    //                    namedWindow("prew", WINDOW_AUTOSIZE);

                    imshow("prew", original);

                    ////////// end of displaying
                    ///////////////////////////////////////////////
                    // fill previous image again
                    img.copyTo(prevgray);
                }


            } else {


                // fill previous image in case prevgray.empty() == true

                if (framesToSkip != 0) {
                    framesToSkip--;
                } else {
                    img.copyTo(prevgray); // finally init prevgray
                }

            }


            int key1 = waitKey(20);
            if (movingObjectWasDetected) break;

        }
    }// END OF INFINITE LOOP OF MOTION DETECTION
}