#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <Eigen/Dense>

int main(int, char**) {
    cv::VideoCapture vcap;
    cv::Mat image;
    cv::Mat lower_resolution;
    Eigen::Matrix<float,3,3> mat;
    mat << 1,2,3,4,5,6,7,8,9;
    std::cout << mat;
    //"rtsp://192.168.1.10:554/user=admin&password=&channel=1&stream=0.sdp?real_stream"
    // doesnt work version
    const std::string videoStreamAddress = "rtsp://192.168.1.222:554/user=admin&password=&channel=1&stream=0.sdp?real_stream";
    //    const std::string videoStreamAddress = "http://<admin:>@<192.168.1.10>/video.cgi?.mjpg";
    //    const std::string videoStreamAddress = "http://ID:PASSWORD@IPADDRESS:PORTNO/mjpeg.cgi?user=ID&password=ID:PASSWORD&channel=0&.mjpg";

    //open the video stream and make sure it's opened
    if (!vcap.open(videoStreamAddress)) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    for (;;) {
        if (!vcap.read(image)) {
            std::cout << "No frame" << std::endl;
            cv::waitKey();
        }
        cv::resize(image, lower_resolution, cv::Size(image.cols * 0.5,image.rows * 0.5), 0, 0, cv::INTER_LANCZOS4);
        cv::imshow("Output Window", lower_resolution);
        if (cv::waitKey(1) >= 0) break;
    }
}