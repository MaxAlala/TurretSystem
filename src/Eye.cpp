/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Eye.cpp
 * Author: sol
 * 
 * Created on January 2, 2021, 11:46 AM
 */

#include "Eye.h"
#include <iostream>
#include <cassert>
#include <opencv2/calib3d/calib3d.hpp>
#include "popt_pp.h"
#include <sys/stat.h>

static cv::Point chosenPoint;
static bool wasPointChosen;
using namespace std;
using namespace cv;

vector< vector< Point3f > > object_points;
vector< vector< Point2f > > image_points;
vector< Point2f > corners;
vector< vector< Point2f > > left_img_points;

Mat img, gray;
Size im_size;
Mat K;
Mat D;
int skipFrame = 0;

double computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints,
        const vector< vector< Point2f > >& imagePoints,
        const vector< Mat >& rvecs, const vector< Mat >& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs) {
    vector< Point2f > imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    vector< float > perViewErrors;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int) objectPoints.size(); ++i) {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
        int n = (int) objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err * err / n);
        totalErr += err*err;
        totalPoints += n;
    }
    return std::sqrt(totalErr / totalPoints);
}

bool doesExist(const std::string& name) {
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

void setup_calibration(int board_width, int board_height, int num_imgs,
        float square_size, char* imgs_directory, char* imgs_filename,
        char* extension) {
    Size board_size = Size(board_width, board_height);
    int board_n = board_width * board_height;

    for (int k = 1; k <= num_imgs; k++) {
        char img_file[100];
        sprintf(img_file, "%s%s%d.%s", imgs_directory, imgs_filename, k, extension);
        if (!doesExist(img_file))
            continue;
        img = imread(img_file, CV_LOAD_IMAGE_COLOR);
        cv::cvtColor(img, gray, CV_BGR2GRAY);

        bool found = false;
        found = cv::findChessboardCorners(img, board_size, corners,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        if (found) {
            cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                    TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray, board_size, corners, found);
        }

        vector< Point3f > obj;
        for (int i = 0; i < board_height; i++)
            for (int j = 0; j < board_width; j++)
                obj.push_back(Point3f((float) j * square_size, (float) i * square_size, 0));

        if (found) {
            cout << k << ". Found corners!" << endl;
            image_points.push_back(corners);
            object_points.push_back(obj);
        }
    }
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        chosenPoint.x = x;
        chosenPoint.y = y;
        wasPointChosen = true;
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
}

void calibrateCamera() {
    setup_calibration(9, 6, 60, 0.0255,
            "../img/", "", "jpg");

    printf("Starting Calibration\n");

    vector< Mat > rvecs, tvecs;
    int flag = 0;
    flag |= CV_CALIB_FIX_K4;
    flag |= CV_CALIB_FIX_K5;
    calibrateCamera(object_points, image_points, img.size(), K, D, rvecs, tvecs, flag);

    cout << "Calibration error: " << computeReprojectionErrors(object_points, image_points, rvecs, tvecs, K, D) << endl;

    char out_file[] = {"cam_left.yml"};
    char extension[] = {"jpg"};

    FileStorage fs(out_file, FileStorage::WRITE);
    fs << "K" << K;
    fs << "D" << D;
    std::cout << "K " << K << "\n";
    std::cout << "D " << D << "\n";
    printf("Done Calibration\n");

}

Eye::Eye(int camera_id, bool shouldFlipFrame_, bool hasMovementDetection_) :
wasPointChosen{false},
selectionWindow{"select a point"},
selectedPointWindow{"selected point"},
shouldFlipFrame{shouldFlipFrame_},
hasMovementDetection{hasMovementDetection_}
{
    movementDetector.reset(new MovementDetector());
    //            cap.open("rtsp://192.168.1.111//user=admin&password=&channel=1&stream=1.sdp?real_stream--rtp-caching=0");
    //    calibrateCamera();

    // read calibration data if already calibrated
    char out_file[] = {"cam_left.yml"};
    FileStorage fs(out_file, FileStorage::READ);
    fs["K"] >> K;
    fs["D"] >> D;
    std::cout << "K " << K << "\n";
    std::cout << "D " << D << "\n";
    printf("Done Calibration\n");
    //


    cap.open(camera_id);
    if (!cap.isOpened()) {
        assert("ERROR! Unable to open camera\n");
    }
    cv::namedWindow(selectionWindow, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(selectedPointWindow, cv::WINDOW_AUTOSIZE);
    //    cv::namedWindow("before", cv::WINDOW_AUTOSIZE);
    int width = 640;
    int height = 480;
    cv::resizeWindow(selectionWindow, width, height);
    cv::resizeWindow(selectedPointWindow, width, height);
    //    cv::resizeWindow("before", width, height);
    //set the callback function for any mouse event
    cv::setMouseCallback(selectionWindow, CallBackFunc, NULL);
}

// returns a selected point + shows cam output

cv::Point Eye::run() {
    cap.read(frame);
    if (frame.empty()) {
        assert("ERROR! blank frame grabbed\n");
    }

    ////////////////////////// START OF DISTORTION FIX
    //    if (shouldFlipFrame)cv::flip(frame, frame, 1);
    //
    //
    ////    cout << frame.size << " ZERO" << endl;
    //
    //
    //
    //    // fix distortion part
    //    //    cv::Size imageSize(frame.size());
    //    cv::Size imageSize(cv::Size(frame.cols, frame.rows));
    //
    //    cv::Mat dst, map1, map2, new_camera_matrix;
    //    // Refining the camera matrix using parameters obtained by calibration
    //
    //    //    cout << frame.size << " ZERO.1" << endl;
    //
    //    new_camera_matrix = cv::getOptimalNewCameraMatrix(K, D, imageSize, 1, imageSize, 0);
    //
    //    //    cout << frame.size << " ONE" << endl;
    //
    //    // Method 1 to undistort the image
    //    cv::undistort(frame, dst, new_camera_matrix, D, new_camera_matrix);
    //
    //    // Method 2 to undistort the image
    //    cv::initUndistortRectifyMap(K, D, cv::Mat(), cv::getOptimalNewCameraMatrix(K, D, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
    //
    //    cv::remap(frame, dst, map1, map2, cv::INTER_LINEAR);
    //    // end of fix distortion
    //    //    cout << frame.size << "output frame size" << endl;
    ////    imshow("before", dst);
    //    frame = dst;


    ///////////////////// END FOR DISTORTION FIX
    cv::circle(frame, cv::Point(320, 240), 10, (51, 102, 255), 2);
    if (shouldFlipFrame)cv::flip(frame, frame, 1);
    //
    //    cv::line(frame,
    //            cv::Point(frame.cols / 4, 0),
    //            cv::Point(frame.cols / 4, 480),
    //            cv::Scalar(0, 0, 0),
    //            3);
    //
    //    cv::line(frame,
    //            cv::Point(frame.cols / 4 * 2, 0),
    //            cv::Point(frame.cols / 4 * 2, 480),
    //            cv::Scalar(0, 0, 0),
    //            3);
    //
    //    cv::line(frame,
    //            cv::Point(frame.cols / 4 * 3, 0),
    //            cv::Point(frame.cols / 4 * 3, 480),
    //            cv::Scalar(0, 0, 0),
    //            3);
    //
    //    cv::line(frame,
    //            cv::Point(0, frame.rows / 4),
    //            cv::Point(640, frame.rows / 4),
    //            cv::Scalar(0, 0, 0),
    //            3);
    //
    //    cv::line(frame,
    //            cv::Point(0, frame.rows / 4 * 2),
    //            cv::Point(640, frame.rows / 4 * 2),
    //            cv::Scalar(0, 0, 0),
    //            3);
    //
    //    cv::line(frame,
    //            cv::Point(0, frame.rows / 4 * 3),
    //            cv::Point(640, frame.rows / 4 * 3),
    //            cv::Scalar(0, 0, 0),
    //            3);


    imshow(selectionWindow, frame);
    if (hasMovementDetection) {

        if(skipFrame != 0)
        {
            --skipFrame;
            std::cout<< skipFrame << std::endl;
            return cv::Point(0, 0);
        }
        
        Point pointToReturn(0, 0);
        movementDetector->findMovement(frame, pointToReturn);
        if (pointToReturn != Point(0, 0))
            skipFrame = 15;
        std::cout << pointToReturn << std::endl;
        return pointToReturn;
    } else
        if (::wasPointChosen) {
        cv::circle(frame, ::chosenPoint, 10, (0, 255, 0), 2);
        imshow(selectedPointWindow, frame);
        ::wasPointChosen = false;

        return ::chosenPoint;
    } else return cv::Point(0, 0);
}

Eye::Eye(const Eye& orig) {
}

Eye::~Eye() {
}

