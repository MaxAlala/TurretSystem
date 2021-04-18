// Copyright (C) 2019 Eugene a.k.a. Realizator, stereopi.com, virt2real team
// Ported from Python to C++ by Konstantin Ozernov on 10/10/2019.
//
// This file is part of StereoPi С++ tutorial scripts, and has been
// ported from Pyton version (https://github.com/realizator/stereopi-fisheye-robot)
//
// StereoPi tutorial is free software: you can redistribute it 
// and/or modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation, either version 3 of the 
// License, or (at your option) any later version.
//
// StereoPi tutorial is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with StereoPi tutorial.  
// If not, see <http://www.gnu.org/licenses/>.
//


#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <ctime>
#include <chrono>
long long getTimestamp()
{
    const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    const std::chrono::microseconds epoch = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
    return  epoch.count();
}

std::string folder_name = "~/git_ws/TurretSystem/bin/files"; 

int main()
{
    int cam_id1 = 1;
    int cam_id2 = 2;
    
    cv::VideoCapture cap1;
    cap1.open(cam_id1);
    
    cv::VideoCapture cap2;
    cap2.open(cam_id2);
    
    std::string winName1 = "leftCam";
    std::string winName2 = "rightCam";
    
    if (!cap1.isOpened()) {
        assert("ERROR! Unable to open camera1\n");
    }
    
    if (!cap2.isOpened()) {
    assert("ERROR! Unable to open camera2\n");
    }
    cv::namedWindow(winName1, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(winName2, cv::WINDOW_AUTOSIZE);
    
//    int width = 640;
//    int height = 480;
//    
//    cv::resizeWindow(winName1, width, height);
//    cv::resizeWindow(winName2, width, height);
    
    cv::Mat frame1;
    cv::Mat frame2;

    fprintf(stderr, "You can press 'Q' to quit this script.\n");
  
    double framesNumber = 0;
    long long startTime = 0;
    long long totalTime = 0;
    int fps = 0;
    int font = cv::FONT_HERSHEY_SIMPLEX;
    time_t start, end;

    while (true)
    {
        cap1.read(frame1);
        cap2.read(frame2);
        bool shouldFlipFrame = true;
        if (shouldFlipFrame)cv::flip(frame1, frame1, 0);
        if (shouldFlipFrame)cv::flip(frame2, frame2, 0);
        double scaler = 0.3;
        cv::line( frame1,
        cv::Point( frame1.cols / 2, 0 ),
        cv::Point( frame1.cols / 2, frame1.rows ),
        cv::Scalar(139, 0, 0),
        3);

        cv::line( frame1,
        cv::Point( 0, frame1.rows / 2),
        cv::Point( frame1.cols, frame1.rows / 2),
        cv::Scalar(139, 0, 0),
        3);

        cv::line( frame2,
        cv::Point( frame2.cols / 2, 0 ),
        cv::Point( frame2.cols / 2, frame2.rows ),
        cv::Scalar(139, 0, 0),
        3);

        cv::line( frame2,
        cv::Point( 0, frame2.rows / 2),
        cv::Point( frame2.cols, frame2.rows / 2),
        cv::Scalar(139, 0, 0),
        3);

        cv::resize(frame1, frame1, cv::Size(), scaler, scaler);
        cv::resize(frame2, frame2, cv::Size(), scaler, scaler);
        if(framesNumber == 0)
        {
         startTime = getTimestamp();
         time(&start);   
        }

        framesNumber++;
	
        char k = cv::waitKey(1);
        if (k == 'q' || k == 'Q')
        {
//            cv::imwrite(folder_name + "frame.jpg", frame);
            break;
        }
        
        if(framesNumber == 99)
        {
            time(&end);   
            double seconds = difftime (end, start);
            fps = framesNumber / seconds ;      
            framesNumber = 0;
            std::cout << seconds << std::endl;
                
            totalTime = getTimestamp();
//            double avgFPS = (totalTime -startTime)/ 1000 / framesNumber;
//                        std::cout << avgFPS << std::endl;
                
//            fprintf(stderr, "Average FPS: %f\n", 1000 / avgFPS);
        }

                
                
        cv::putText(frame1, std::to_string(fps), cv::Point(50,50), font, 2.0, cv::Scalar(0,0,255), 4, 16 /*CV_AA*/);
        cv::putText(frame2, std::to_string(fps), cv::Point(50,50), font, 2.0, cv::Scalar(0,0,255), 4, 16 /*CV_AA*/);
        cv::imshow(winName1, frame1);
        cv::imshow(winName2, frame2);
    }

    return 0;
}
