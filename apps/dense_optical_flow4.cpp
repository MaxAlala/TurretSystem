//
//// Copyright (C) 2019 Eugene a.k.a. Realizator, stereopi.com, virt2real team
//// Ported from Python to C++ by Konstantin Ozernov on 10/10/2019.
////
//// This file is part of StereoPi С++ tutorial scripts, and has been
//// ported from Pyton version (https://github.com/realizator/stereopi-fisheye-robot)
////
//// StereoPi tutorial is free software: you can redistribute it 
//// and/or modify it under the terms of the GNU General Public License
//// as published by the Free Software Foundation, either version 3 of the 
//// License, or (at your option) any later version.
////
//// StereoPi tutorial is distributed in the hope that it will be useful,
//// but WITHOUT ANY WARRANTY; without even the implied warranty of
//// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//// GNU General Public License for more details.
////
//// You should have received a copy of the GNU General Public License
//// along with StereoPi tutorial.  
//// If not, see <http://www.gnu.org/licenses/>.
////
//
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
//#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>
//long long getTimestamp()
//{
//   const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
//   const std::chrono::microseconds epoch = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
//   return  epoch.count();
//}

int main() {
    using namespace cv;
    using namespace std;
    int cam_id1 = 1;
    cv::VideoCapture cap1;
    cap1.open(cam_id1);
    //    
    //    cv::VideoCapture cap2;
    //    cap2.open(cam_id2);

    std::string winName1 = "leftCam";
    //    std::string winName2 = "rightCam";

    if (!cap1.isOpened()) {
        assert("ERROR! Unable to open camera1\n");
    }
    //    
    //    if (!cap2.isOpened()) {
    //    assert("ERROR! Unable to open camera2\n");
    //    }
    cv::namedWindow(winName1);
    //    cv::namedWindow(winName2, cv::WINDOW_AUTOSIZE);

    int width = 640;
    int height = 480;

    cv::resizeWindow(winName1, width, height);
    //    cv::resizeWindow(winName2, width, height);

    cv::Mat frame1;

    // donwload a specific image
    std::string image_path = ("savedRngFnder.png");
    frame1 = cv::imread(image_path, cv::IMREAD_COLOR);
    cv::Mat grayscaled;
    // preprocessing
    cv::cvtColor(frame1, grayscaled, cv::COLOR_BGR2GRAY);


    // select + shows specific ROI 
    // all 3 digits take
    // 120 x
    // 70 y
    // x/3 = 40 pixels per digit
    //     cv::Rect2d roiRect = cv::selectROI(frame1);
    //     std::cout << roiRect.x << " " << roiRect.y << std::endl;
    //     std::cout << roiRect.x + roiRect.width << " " << roiRect.y + roiRect.height << std::endl;


    cv::Rect2d roiRect(cv::Point(247, 364), cv::Point(345, 418));
    int xPlus = 26;
    int yPlus = 54;
    int spaceBetween = 9;
    cv::Mat roiImg = grayscaled(roiRect);
    roiRect = Rect2d(Point(roiRect.x, roiRect.y), Point(roiRect.x + xPlus, roiRect.y + yPlus));


    cv::Mat digit1 = grayscaled(roiRect).clone();

    double x_vertical_width = 0.25;
    //     double height
    vector<Rect2d> segments ={
        {Point(0, 0), Point(roiRect.width, roiRect.height * 0.15)}, // top
        {Point(0, 0), Point(roiRect.width*x_vertical_width, roiRect.height / 2)}, // top-left
        {Point(roiRect.width - roiRect.width*x_vertical_width, 0), Point(roiRect.width, roiRect.height / 2)}, // top-right
        {Point(0, roiRect.height / 2 - roiRect.height * 0.075), Point(roiRect.width, roiRect.height / 2 + roiRect.height * 0.075)}, // middle
        {Point(0, roiRect.height / 2), Point(roiRect.width*x_vertical_width, roiRect.height)}, // bottom-left
        {Point(roiRect.width - roiRect.width * x_vertical_width, roiRect.height / 2), Point(roiRect.width, roiRect.height)}, // bottom-right
        {Point(0, roiRect.height - roiRect.height * 0.15), Point(roiRect.width, roiRect.height)}, // bottom
    };

    roiRect = Rect2d(Point(roiRect.x + xPlus + spaceBetween, roiRect.y), Point(roiRect.x + xPlus * 2 + spaceBetween, roiRect.y + yPlus));
    cv::Mat digit2 = grayscaled(roiRect).clone();

    roiRect = Rect2d(Point(roiRect.x + xPlus + spaceBetween, roiRect.y), Point(roiRect.x + xPlus * 2 + spaceBetween, roiRect.y + yPlus));
    cv::Mat digit3 = grayscaled(roiRect).clone();

    vector<Mat> segmentsMats(7);
    vector<bool> isThereSegment(7);

    //     
    for (int x = 0; x < 7; x++) {
//    {
//    int x = 1;
        Mat segment = digit3(segments.at(x)).clone();
        cv::threshold(segment, segment, 100, 255, cv::THRESH_BINARY);
        segmentsMats.at(x) = segment.clone();
        //       cv::threshold(segment, segment, 80, 255, cv::THRESH_BINARY_INV);
        int numberOfWhitePixels = 0;
        int pixelNumber = segment.cols * segment.rows;
        for (int i = 0; i < segment.cols; ++i)
            for (int j = 0; j < segment.rows; ++j) {
                //               cout <<(int)segment.at<uchar>(j,i) << endl; 
                if (segment.at<uchar>(j, i) == 0)
                    ++numberOfWhitePixels;
            }
//        for (int x = 0; x < 3; x++) {

        cv::imshow(string(to_string(x) + "seg"), segmentsMats.at(x));
//        }
        
        isThereSegment.at(x) = (double)numberOfWhitePixels / pixelNumber > 0.20 ? true:false;
        
            
        cout << "x = " << x << ", segment number of pixels = " << pixelNumber << endl;
        cout << "numberOfWhitePixels  = " << numberOfWhitePixels << endl;
    }
    //     if> 10% = there is segment


    for (int i = 0; i < isThereSegment.size(); ++i)
    {
       cout << " is there segment = " << isThereSegment.at(i) << endl; 
    }



//    vector<bool> isThereSegment(7, 0);




    //     int blur = 4;
    //     cv::GaussianBlur(roiImg, roiImg,cv::Size(blur,blur),0);
    cv::Mat contoursImg(roiImg.clone());
    cv::Mat thresholded;
    cv::threshold(roiImg, thresholded, 80, 255, cv::THRESH_BINARY_INV);




    cv::threshold(contoursImg, contoursImg, 0, 255, cv::THRESH_BINARY_INV);

    int dilation_size = 5;
    Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
            Size(dilation_size, dilation_size));


    dilate(thresholded, thresholded, element);


    //            morphologyEx( thresholded, thresholded, MORPH_OPEN, element );



    std::vector<std::vector<cv::Point> > contours;
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());

    std::vector<cv::Vec4i> hierarchy;
    findContours(thresholded, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    //    cv::RNG rng(12345);

    for (size_t i = 0; i < contours.size(); i++) {
        //        cv::Rect rect;
        //        approxPolyDP( contours[i], contours_poly[i], 3, true );
        //        boundRect[i] = boundingRect( contours_poly[i] );
        //        rect = boundingRect(contours.at(i));
        //        std::cout << boundRect[i].width <<  " " << boundRect[i].height << " \n";
        cv::Scalar color = (0, 0, 125);
        drawContours(contoursImg, contours, (int) i, color, 2, cv::LINE_8, hierarchy, 0);
        //        rectangle(roiImg, Point(boundRect[i].x, boundRect[i].y), Point(boundRect[i].x+boundRect[i].width, boundRect[i].y+boundRect[i].height), Scalar(0,200,0),1);


    }

    //         if(frame1.empty())
    //    {
    //        std::cout << "Could not read the image: " << image_path << std::endl;
    //        return 1;
    //    }
    while (true) {
                cap1.read(frame1);
        //        cap2.read(frame2);
        bool shouldFlipFrame = true;
                if (shouldFlipFrame)cv::flip(frame1, frame1, 0);
        //        if (shouldFlipFrame)cv::flip(frame2, frame2, 0);
        //        double scaler = 1;
        //        cv::line( frame1,
        //        cv::Point( frame1.cols / 2, 0 ),
        //        cv::Point( frame1.cols / 2, frame1.rows ),
        //        cv::Scalar(139, 0, 0),
        //        3);
        //
        cv::line(frame1,
                cv::Point(0, frame1.rows / 2),
                cv::Point(frame1.cols, frame1.rows / 2),
                cv::Scalar(139, 0, 0),
                3);
        //
        //        cv::rectangle(frame1, cv::Point(310, 360), cv::Point(340, 420), cv::Scalar(125,0,0));
        //          cv::rectangle(frame1, cv::Point(250, 360), cv::Point(340, 420), cv::Scalar(0,125,0));


        //        cv::line( frame2,
        //        cv::Point( frame2.cols / 2, 0 ),
        //        cv::Point( frame2.cols / 2, frame2.rows ),
        //        cv::Scalar(139, 0, 0),
        //        3);
        //
        //        cv::line( frame2,
        //        cv::Point( 0, frame2.rows / 2),
        //        cv::Point( frame2.cols, frame2.rows / 2),
        //        cv::Scalar(139, 0, 0),
        //        3);

        //        cv::resize(frame1, frame1, cv::Size(), scaler, scaler);
        //        cv::resize(frame2, frame2, cv::Size(), scaler, scaler);
        //        if(framesNumber == 0)
        //        {
        //         startTime = getTimestamp();
        //         time(&start);   
        //        }

        //        framesNumber++;

        char k = cv::waitKey(1);
        if (k == 'q' || k == 'Q') {
            cv::imwrite("savedRngFnder.png", frame1);
            //	    cv::imwrite(rightName, frame2);
            //            cv::imwrite(folder_name + "frame.jpg", frame);
            break;
        }
        //        
        //        if(framesNumber == 99)
        //        {
        //            time(&end);   
        //            double seconds = difftime (end, start);
        //            fps = framesNumber / seconds ;      
        //            framesNumber = 0;
        //            std::cout << seconds << std::endl;
        //                
        //            totalTime = getTimestamp();
        ////            double avgFPS = (totalTime -startTime)/ 1000 / framesNumber;
        ////                        std::cout << avgFPS << std::endl;
        //                
        ////            fprintf(stderr, "Average FPS: %f\n", 1000 / avgFPS);
        //        }


        //                
        //        cv::putText(frame1, std::to_string(fps), cv::Point(50,50), font, 2.0, cv::Scalar(0,0,255), 4, 16 /*CV_AA*/);
        //        cv::putText(frame2, std::to_string(fps), cv::Point(50,50), font, 2.0, cv::Scalar(0,0,255), 4, 16 /*CV_AA*/);
        cv::imshow(winName1, roiImg);
        cv::imshow("contours", contoursImg);
        cv::imshow("thresholded", thresholded);
//        cv::imshow("", frame1);

        //        
        cv::imshow("1", digit1);
        cv::imshow("2", digit2);
        cv::imshow("3", digit3);
        cv::imshow("frame1", frame1);
    }


    fprintf(stderr, "Photo sequence finished\n");

    return 0;
}

