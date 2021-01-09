


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

using namespace cv;
using namespace std;

int main(int argc, const char** argv) {

    // add your file name

    VideoCapture cap(1);
    //        namedWindow("prew", WINDOW_AUTOSIZE);
    Mat m;
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    for (;;) {


        cap.read(m);
        if (m.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        //        circle(m, Point(325, 325), 30, Scalar(0, 0, 255), 5);
        //                rectangle(original, rect, Scalar(255, 0, 0), 2, 1);
        // draw the results

        imshow("prew222", m);

        //    char modelWasChoosen = ' ';
        //    while (modelWasChoosen != 'Y' && modelWasChoosen != 'N') {
        //        std::cout << "Error invalid input please try again " << std::endl;
        //        std::cout << "Please enter [Y/N] if u or not to send detected motion: ";
        //        std::cin >> modelWasChoosen;
        //        modelWasChoosen = toupper(modelWasChoosen);
        //    }
        //
        //    std::cout << "u have pressed: " << modelWasChoosen;
        //    if (modelWasChoosen == 'Y') {
        //         circle(m, Point(325, 500), 30, Scalar(0, 0, 255), 5);
        //    }
        //         imshow("prew", m);
        if (waitKey(5) >= 0)
            break;
    }

    return 0;
}
