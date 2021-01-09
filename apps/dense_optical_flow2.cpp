


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
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <thread>
using namespace cv;
using namespace std;
int currentGroupNumber = 1;
vector<int > groupCount(10);
const int x_divider = 10;
const int y_divider = 10;
//calculate area motion weight of 18 areas == chose the best one
vector<int > motionAreaWeights(x_divider * y_divider, 0);
int x_resolution = 480;
int y_resolution = 640;
int x_pixels_in_cell = x_resolution / x_divider;
int y_pixels_in_cell = y_resolution / y_divider;

int weightedCells [x_divider][y_divider];
int groupedCells [x_divider][y_divider];
Point detectedObject;
vector<Point > groupAverage(10);

vector<Point> topLeft(10);
vector<Point> bottomRight(10);
int robotCam = 1;



int focal_length = 492; // width/f = tg a // a == 33. width = 320
double angleX = 0;
double angleY = 0;
vector<int> stepsPer1Angle(6);
vector<int> axisSpeeds(6);

void init() {
    stepsPer1Angle[3] = 111;
    stepsPer1Angle[4] = 104;
    axisSpeeds[3] = 800;
    axisSpeeds[4] = 800;

}

double removeSign(double x) {
    return sqrt(x * x);
}

int fromRadToGrad(double x) {
    return x * 180 / 3.1415;
}

void calculateAngles(Point& detectedObject) {
    angleX = fromRadToGrad(atan((removeSign(x_resolution / 2 - detectedObject.x) / focal_length)));
    angleY = fromRadToGrad(atan((removeSign(y_resolution / 2 - detectedObject.y) / focal_length)));
    if (detectedObject.x > x_resolution / 2) angleX = -angleX;
    if (detectedObject.y > y_resolution / 2) angleY = -angleY;
    cout << "calculated x y angles to reach position: " << angleX << " " << angleY << endl;
}

void checkNeighbourhood(int i, int j) {

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

void printGroupCellMatrix() {
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

void setDefaultGroupCellMatrix() {
    cout << "cell group matrix: \n";
    for (int i = 0; i < y_divider; i++) {
        for (int j = 0; j < x_divider; j++) {
            groupedCells[i][j] = 0;
            weightedCells[i][j] = 0;
        }
        cout << "\n";
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
        groupAverage.at(i) = Point(0, 0);
    }
}

void findMotion(VideoCapture& cap, Point& detectedObject) {
        Mat flow;
    int framesToSkip = 0;
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
            if (robotCam)flip(img, img, 1);
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
                    
                    //divides a frame into parts
                    for (int i = 0; i < x_divider; i++) 
                    {
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
                            // if detected a cell without a group == creates a new group
                            if ((weightedCells[i][j] != 0) && (groupedCells[i][j] == 0)) 
                            {
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
                    cout << "groups count: \n";
                    for (int i = 0; i < groupCount.size(); i++) {
                        cout << groupCount.at(i) << " ";
                    }
                    cout << "\n";

                    cout << "MAX group is " << maxGroup << endl;
                    detectedObject = Point(groupAverage[maxGroup].x * x_pixels_in_cell / groupCount[maxGroup], groupAverage[maxGroup].y * y_pixels_in_cell / groupCount[maxGroup]);

                    cout << "detected Object: " << detectedObject.x << " " << detectedObject.y << endl;


                    calculateAngles(detectedObject);
                    //////////////////////////// SENDING PART
                    string str;
                    string message = "r 0 0 0 0 0 0 " + to_string(angleY * stepsPer1Angle[3]) + " " + to_string(axisSpeeds[3]) + " " + to_string(angleX * stepsPer1Angle[4]) + " " + to_string(axisSpeeds[4]) + "\n";

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
                    int key1 = waitKey(21);
                    ////////// end of displaying
                    //                    std::cout << "u have pressed: " << modelWasChoosen;
                    //                    if (modelWasChoosen == 'Y') {
                    serial_port.Write(message); // go up
                    //                      serial_port.Write("mca 4 200 0\n"); // go up
                    cout << "THIS MESSAGE WAS SENT: " << message << "\n";
                    ;
                    while (str.substr(0, 7) != "running") {
                        str = "";
                        if (serial_port.GetNumberOfBytesAvailable() != 0) serial_port.ReadLine(str);
                        if (str != "") cout << str << "\n";
//                        cout << str.substr(0, 7) << endl;
                    }
                    //                         str = "";
                    cout << "sending was finished. \n";
                    //                    }
                    //////////////////////// END OF SENDING PART

                    setDefaultGroupCellMatrix();
                    // make prevgray empty to get else section
                    prevgray = UMat();
                    framesToSkip = 5; 
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

                if(framesToSkip != 0)
                {
                    framesToSkip--;
                }else
                {
                img.copyTo(prevgray); // finally init prevgray
                }

            }


            int key1 = waitKey(20);
            if (movingObjectWasDetected) break;

        }
    }// END OF INFINITE LOOP OF MOTION DETECTION
}

void checkMotionInSpecificRectangle(VideoCapture& cap, Rect2d& rect, int& rectangleNum) {
    Mat flow, frame;

    // some faster than mat image container
    UMat flowUmat, prevgray;

    Point topLeftAreaPoint;
    Point bottomRightAreaPoint;
    bool movingObjectWasDetected = 0;

    Mat original;
    for (;;) {

        bool Is = cap.grab();

        if (Is == false) {


            // if video capture failed

            cout << "Video Capture Fail" << endl;


            break;


        } else {

            Mat img;

            // capture frame from video file

            cap.retrieve(img, CV_CAP_OPENNI_BGR_IMAGE);


            resize(img, img, Size(640, 480));


            // save original for later

            img.copyTo(original);


            // just make current frame gray

            cvtColor(img, img, COLOR_BGR2GRAY);


            if (prevgray.empty() == false) {

                // calculate optical flow 


                calcOpticalFlowFarneback(prevgray, img, flowUmat, 0.4, 1, 12, 2, 8, 1.2, 0);

                   // copy Umat container to standard Mat

                flowUmat.copyTo(flow);

                bool needToCheckMotionArea = 1;
                //calculate area motion weight of 18 areas == chose the best one
                vector<int > motionAreaWeights(1, 0);
                vector<Point > topLeftAreaCoordinatesForRect(18, Point(0, 0));
                int numberOf_X_PixelsInOneArea = 106;
                int numberOf_Y_PixelsInOneArea = 160;
                if (needToCheckMotionArea) {
                    int length = 0;
                    for (int x = rect.x; x < rect.x + numberOf_X_PixelsInOneArea; x++) {
                        for (int y = rect.y; y < rect.y + numberOf_Y_PixelsInOneArea; y++) {
                            const Point2f flowatxy = flow.at<Point2f>(y, x) * 100;

                            Point difference(y - (y + flowatxy.y), x - (x + flowatxy.x));
                            length = sqrt(difference.x * difference.x + difference.y * difference.y);
                            //                                    cout << length << "length \n";
                            //                                    if (i == 5 && j == 1){
                            //                                    if (i == 0 && j == 0) {
                            //                                        cout << "pixel xy:" << x << " " << y << " . flow xy is: " << flowatxy.x << " " << flowatxy.y << ". length:" << length << endl;
                            //
                            //                                    }
                            if (length > 1000) motionAreaWeights.at(0) += length;
                        }
                    }

                    if (length != 0)movingObjectWasDetected = true;
                    cout << "is movement was detected in rectangle num " << rectangleNum << ", " << movingObjectWasDetected << endl;
                    //                    rectangle(original, rect, Scalar(255, 0, 0), 2, 1);
                }

                // draw the results
                namedWindow("prew", WINDOW_AUTOSIZE);

                imshow("prew", original);

                // fill previous image again
                img.copyTo(prevgray);

            } else {

                // fill previous image in case prevgray.empty() == true

                img.copyTo(prevgray);
            }

            int key1 = waitKey(20);
            if (movingObjectWasDetected) break;

        }// END OF INFINITE LOOP OF MOTION DETECTION
    }
}

int main(int argc, const char** argv) {

    // add your file name
    init();
    VideoCapture cap(1);
    Point detectedObject;
    findMotion(cap, detectedObject);
    //    makeTracking(cap, rect);

}

