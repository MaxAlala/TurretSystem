/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MovementDetector.cpp
 * Author: sol
 * 
 * Created on January 14, 2021, 7:09 PM
 */

#include "MovementDetector.h"

int currentGroupNumber = 1;
std::vector<int > groupCount(10);
const int x_divider = 10;
const int y_divider = 10;
//calculate area motion weight of 18 areas == chose the best one
std::vector<int > motionAreaWeights(x_divider * y_divider, 0);
int x_resolution = 480;
int y_resolution = 640;
int x_pixels_in_cell = x_resolution / x_divider;
int y_pixels_in_cell = y_resolution / y_divider;

int weightedCells [x_divider][y_divider];
int groupedCells [x_divider][y_divider];
std::vector<cv::Point > groupAverage(10);
std::vector<cv::Point> topLeft(10);
std::vector<cv::Point> bottomRight(10);
int robotCam = 1;
int framesToSkip = 0;
cv::Mat flow;
std::string winName = "detectedMovement";
cv::Point detectedObject(0,0);
// some faster than mat image container
cv::UMat flowUmat, prevgray;
int minLengthSizeOfSubtractionVector = 1500;

//    Point topLeftAreaPoint;
//    Point bottomRightAreaPoint;
int movingObjectWasDetected = 0;
cv::Mat original;

void MovementDetector::checkNeighbourhood(int i, int j) {

    // check if a cell is within the bounds + there was a movement in a cell
    // + group cell is not busy
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

MovementDetector::MovementDetector() {
    cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);
}

MovementDetector::MovementDetector(const MovementDetector& orig) {
}

MovementDetector::~MovementDetector() {
}

void printGroupCellMatrix() {
    std::cout << "cell group matrix: \n";
    for (int i = 0; i < x_divider; i++) {
        for (int j = 0; j < y_divider; j++) {
            std::cout << groupedCells[i][j] << " ";
        }
        std::cout << "\n";
    }


    std::cout << "cell weight matrix: \n";
    for (int i = 0; i < x_divider; i++) {
        for (int j = 0; j < y_divider; j++) {
            std::cout << weightedCells[i][j] << " ";
        }
        std::cout << "\n";
    }
}

void setDefaultGroupCellMatrix() {
    std::cout << "cell group matrix: \n";
    for (int i = 0; i < y_divider; i++) {
        for (int j = 0; j < x_divider; j++) {
            groupedCells[i][j] = 0;
            weightedCells[i][j] = 0;
        }
        std::cout << "\n";
    }
    for (int i = 0; i < groupCount.size(); i++) {
        groupCount.at(i) = 0;
    }

    currentGroupNumber = 1;
    //    for (int i = 0; i < groupCount.size(); i++) {
    //        topLeft.at(i) = Point(0, 0);
    //        bottomRight.at(i) = Point(0, 0);
    //    }

    for (int i = 0; i < groupAverage.size(); i++) {
        groupAverage.at(i) = cv::Point(0, 0);
    }
}

void MovementDetector::findMovement(cv::Mat& img, cv::Point& detectedObjectToReturn) {
    img.copyTo(original);

    // just make current frame gray
    cvtColor(img, img, cv::COLOR_BGR2GRAY);

    // For all optical flow you need a sequence of images.. Or at least 2 of them. Previous 
    // and current frame
    // if there is no current frame
    // go to this part and fill previous frame
    // else {
    // img.copyTo(prevgray);
    //   }
    // if previous frame is not empty.. There is a picture of previous frame. Do some
    // optical flow alg. 
    std::cout << "here1 \n";
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
                const cv::Point2f flowatxy = flow.at<cv::Point2f>(y, x) * vector_multiplier;

                // draw line at flow direction, first point is pixel coordinate, second pixels is a flow direction coordinate
                line(original, cv::Point(x, y), cv::Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), cv::Scalar(255, 0, 0));

                // draw initial point
                circle(original, cv::Point(x, y), 1, cv::Scalar(0, 0, 0), -1);
            }
        }
        bool needToCheckMotionArea = 1;
        bool wasMovementDetected = false;
        if (needToCheckMotionArea) {

            //divides a frame into parts
            for (int i = 0; i < x_divider; i++) {
                for (int j = 0; j < y_divider; j++) {
                    //goes through all points of a current cell
                    for (int x = x_pixels_in_cell * i; x < x_pixels_in_cell * (i + 1); x++) {
                        for (int y = y_pixels_in_cell * j; y < y_pixels_in_cell * (j + 1); y++) {

                            //returns a point describing orientation and speed
                            const cv::Point2f flowatxy = flow.at<cv::Point2f>(x, y) * 100;

                            // creates parameters describing size of a motion
                            cv::Point difference(x - (x + flowatxy.x), y - (y + flowatxy.y));
                            int length = sqrt(difference.x * difference.x + difference.y * difference.y);
                            if (length > minLengthSizeOfSubtractionVector) weightedCells[i][j] += length;
                        }
                    }
                }
            }
            // creates motion groups
            for (int i = 0; i < x_divider; i++)
                for (int j = 0; j < y_divider; j++) {
                    // if detected a cell without a group == creates a new group
                    if ((weightedCells[i][j] != 0) && (groupedCells[i][j] == 0)) {
                        wasMovementDetected = true;
                        checkNeighbourhood(i, j);
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
            std::cout << "groups count: \n";
            for (int i = 0; i < groupCount.size(); i++) {
                std::cout << groupCount.at(i) << " ";
            }
            std::cout << "\n";

            std::cout << "MAX group is " << maxGroup << std::endl;
            detectedObject = cv::Point(groupAverage[maxGroup].x * x_pixels_in_cell / groupCount[maxGroup], groupAverage[maxGroup].y * y_pixels_in_cell / groupCount[maxGroup]);

            std::cout << "detected Object: " << detectedObject.y << " " << detectedObject.x << std::endl;

            circle(original, cv::Point(detectedObject.y, detectedObject.x), 30, cv::Scalar(0, 0, 255), 5);
            //                rectangle(original, rect, Scalar(255, 0, 0), 2, 1);
            // draw the results
            //                    namedWindow("prew", WINDOW_AUTOSIZE);

            imshow(winName, original);
            cv::waitKey(5);

            setDefaultGroupCellMatrix();
            prevgray = cv::UMat();
            framesToSkip = 5;
            detectedObjectToReturn = cv::Point(detectedObject.y, detectedObject.x);
        } else {

            circle(original, cv::Point(detectedObject.y, detectedObject.x), 30, cv::Scalar(0, 0, 255), 5);
            //                rectangle(original, rect, Scalar(255, 0, 0), 2, 1);
            // draw the results
            //                    namedWindow("prew", WINDOW_AUTOSIZE);

            imshow(winName, original);

            ////////// end of displaying
            ///////////////////////////////////////////////
            // fill previous image again
            img.copyTo(prevgray);
        }


    } else {

        // fill previous image in case prevgray.empty() == true
        img.copyTo(prevgray); // finally init prevgray

    }


    int key1 = cv::waitKey(20);


}// END OF INFINITE LOOP OF MOTION DETECTION
