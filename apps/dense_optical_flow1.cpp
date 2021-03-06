


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


using namespace cv;


using namespace std;

int robotCam = 1;
void findMotion(VideoCapture& cap, Rect2d& rect);

void makeTracking(VideoCapture& cap, Rect2d& rect) {

    // Exit if video is not opened
    if (!cap.isOpened()) {
        cout << "Could not read video file" << endl;
        exit(0);
    }

    // Read first frame 
    Mat frame;

    Size size(640, 480);
    bool ok = cap.read(frame);
    if (robotCam)flip(frame, frame, 1);
    resize(frame, frame, size);
    ////////////////////////////////////////////////////////////////// TRACKER PARAMETERS
    string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
    // vector <string> trackerTypes(types, std::end(types));
    bool isRectWasSelected = false;
    // Create a tracker
    string trackerType = trackerTypes[1];

    Ptr<Tracker> tracker;

#if (CV_MINOR_VERSION < 3)
    {
        //        tracker = Tracker::create(trackerType);
    }
#else
    {
        if (trackerType == "BOOSTING")
            tracker = TrackerBoosting::create();
        if (trackerType == "MIL")
            tracker = TrackerMIL::create();
        if (trackerType == "KCF")
            tracker = TrackerKCF::create();
        if (trackerType == "TLD")
            tracker = TrackerTLD::create();
        if (trackerType == "MEDIANFLOW")
            tracker = TrackerMedianFlow::create();
        if (trackerType == "GOTURN")
            tracker = TrackerGOTURN::create();
        if (trackerType == "MOSSE")
            tracker = TrackerMOSSE::create();
        //        if (trackerType == "CSRT")
        //            tracker = TrackerCSRT::create();
    }
#endif

    tracker->init(frame, rect);
    int x = 0;

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
    int AxisWasSlowedDown [] = {0, 0};
    /////////////////////////////END
    // VideoCapture video(0);
    string str;
    int whichCommandIsRunningNow[] = {0, 0}; // 4 and 5 axis 
    ////////////////////////////////////////////////////////////////// TRACKER PARAMETERS END



    // TRACK RECTANGLE, CHECK MOTION IN RECTANGLE EVERY 10 FRAME
    // IF THERE IS MOTION SWITCH TO THE MOTION DETECTION TO FIND NEW RECTANGLE
    while (cap.read(frame)) {
        if (robotCam)flip(frame, frame, 1);
        resize(frame, frame, size);

        // Start timer
        //        double timer = (double) getTickCount();

        x++;
        if (x > 2) {
            // Update the tracking result
            bool ok = tracker->update(frame, rect);
            x = 0;
        }
        // Calculate Frames per second (FPS)
        //        float fps = getTickFrequency() / ((double) getTickCount() - timer);


        ////////////////////// X
        //        cout <<"x: "<< rect.x << " " << frame.cols / 2 << endl;
        // IF WE ARE ON THE LEFT SIDE OF THE OBJECT
        if (rect.x + rect.width / 2 < frame.cols / 2 - steps) {
            // slow motion when some border was approached
            if (rect.x + rect.width / 2 < frame.cols / 2 - steps && rect.x + rect.width / 2 > frame.cols / 2 - stepsForMotionSlowing) {
                if (AxisWasSlowedDown[1] == 1) {
                }// if a speed already was slowed down do nothing
                else { // slow down an axis
                    speeds[1] = 200;
                    whichCommandIsRunningNow[1] = 0; // to get access into next else {} to change speed
                    AxisWasSlowedDown[1] = 1;
                    cout << "SLOWING DOWN from left \n";
                }
            } else // if not in the slow area set big speed
            {
                if (AxisWasSlowedDown[1] == 1)whichCommandIsRunningNow[1] = 0; // to send an increased speed
                speeds[1] = bigSpeed;
                AxisWasSlowedDown[1] = 0;
            }

            if (whichCommandIsRunningNow[1] == 1) {
            } else { // slow down or start moving axis in right direction
                serial_port.Write("mca 4 " + to_string(speeds[1]) + " 1\n"); // go up
                //                      serial_port.Write("mca 4 200 0\n"); // go up
                cout << "less 4\n";
                whichCommandIsRunningNow[1] = 1;
            }

            // IF WE ARE ON THE RIGHT SIDE OF THE OBJECT
        } else if (rect.x + rect.width / 2 > frame.cols / 2 + steps) {
            // slow motion when some border was approached
            if (rect.x + rect.width / 2 > frame.cols / 2 + steps && rect.x + rect.width / 2 < frame.cols / 2 + stepsForMotionSlowing) {
                if (AxisWasSlowedDown[1] == 1) {
                } else {
                    speeds[1] = 200;
                    whichCommandIsRunningNow[1] = 0; // to get access into next else {} to change speed
                    AxisWasSlowedDown[1] = 1;
                    cout << "SLOWING DOWN from right\n";
                }
            } else // if not in the slow area set big speed
            {
                if (AxisWasSlowedDown[1] == 1)whichCommandIsRunningNow[1] = 0; // to send an increased speed
                speeds[1] = bigSpeed;
                AxisWasSlowedDown[1] = 0;
            }

            if (whichCommandIsRunningNow[1] == -1) {
            } else {
                serial_port.Write("mca 4 " + to_string(speeds[1]) + " -1\n"); // go down 
                //            serial_port.Write("mca 4 200 0\n"); // go up
                cout << "more 4\n";
                whichCommandIsRunningNow[1] = -1;
            }
        } else {
            if (whichCommandIsRunningNow[1] == 0) {
            } else {
                serial_port.Write("mca 4 0 -1\n"); // stop
                cout << "stop \n";
                whichCommandIsRunningNow[1] = 0;
            }
        }
        //// end x

        //// Y
        //                        cout <<"y: "<< rect.y << " " << frame.rows / 2 << endl;
        //         camera view is above an object
        if (rect.y + rect.height / 2 < frame.rows / 2 - steps) {

            // slow motion when some border was approached
            if (rect.y + rect.height / 2 < frame.rows / 2 - steps && rect.y + rect.height / 2 > frame.rows / 2 - stepsForMotionSlowing) {
                if (AxisWasSlowedDown[0] == 1) {
                }// if a speed already was slowed down do nothing
                else { // slow down an axis
                    speeds[0] = 200;
                    whichCommandIsRunningNow[0] = 0; // to get access into next else {} to change speed
                    AxisWasSlowedDown[0] = 1;
                    cout << "SLOWING DOWN Y from TOP \n";
                }
            } else // if not in the slow area set big speed
            {
                if (AxisWasSlowedDown[0] == 1)whichCommandIsRunningNow[0] = 0; // to send an increased speed == before an axis was slowed down
                speeds[0] = 800;
                AxisWasSlowedDown[0] = 0;
            }

            if (whichCommandIsRunningNow[0] == -1) {
            } else {
                serial_port.Write("mca 5 " + to_string(speeds[0]) + " -1\n"); // go up
                cout << "less 5\n";
                whichCommandIsRunningNow[0] = -1;
            }
            // camera view is below an object
        } else if (rect.y + rect.height / 2 > frame.rows / 2 + steps) {


            // slow motion when some border was approached
            if (rect.y + rect.height / 2 > frame.rows / 2 + steps && rect.y + rect.height / 2 < frame.rows / 2 + stepsForMotionSlowing) {
                if (AxisWasSlowedDown[0] == 1) {
                }// if a speed already was slowed down do nothing
                else { // slow down an axis
                    speeds[0] = 200;
                    whichCommandIsRunningNow[0] = 0; // to get access into next else {} to change speed
                    AxisWasSlowedDown[0] = 1;
                    cout << "SLOWING DOWN Y from TOP \n";
                }
            } else // if not in the slow area set big speed
            {
                if (AxisWasSlowedDown[0] == 1)whichCommandIsRunningNow[0] = 0; // to send an increased speed == before an axis was slowed down
                speeds[0] = 800;
                AxisWasSlowedDown[0] = 0;
            }

            if (whichCommandIsRunningNow[0] == 1) {
            } else {
                serial_port.Write("mca 5 " + to_string(speeds[0]) + " 1\n"); // go down 
                cout << "more 5\n";
                whichCommandIsRunningNow[0] = 1;
            }
        } else {
            if (whichCommandIsRunningNow[0] == 0) {
            } else {
                serial_port.Write("mca 5 0 1\n"); // stop
                cout << "stop 5\n";
                whichCommandIsRunningNow[0] = 0;
            }
        }

        // if the turret isnt moving == probably an object was missed == need to find it again
        if (rect.x + rect.width / 2 >= frame.cols / 2 - steps
                && rect.x + rect.width / 2 <= frame.cols / 2 + steps
                && rect.y + rect.height / 2 >= frame.rows / 2 - steps
                && rect.y + rect.height / 2 <= frame.rows / 2 + steps) {
            //            whichCommandIsRunningNow[0] = 0;
            //            whichCommandIsRunningNow[1] = 0;
            //            serial_port.Write("mca 5 " + to_string(speeds[0]) + " 0\n"); // go down 
            //            serial_port.Write("mca 4 " + to_string(speeds[0]) + " 0\n"); // go down 
            findMotion(cap, rect);
            cout << rect.x << " " << rect.y << " CURRENT RECT \n";

            tracker.release();
            tracker = TrackerMIL::create();
            tracker->init(frame, rect);
            tracker->update(frame, rect);
            cout << rect.x << " " << rect.y << " CURRENT RECT2 \n";
        }
        cout << rect.x << " " << rect.y << " CURRENT RECT3 \n";
        ///////////// end Y
        //        cout << serial_port.GetNumberOfBytesAvailable() << "bytes \n";


        if (serial_port.GetNumberOfBytesAvailable() == 0) {
        } else serial_port.ReadLine(str);

        if (str == "") {
        } else cout << str << "\n";

        str = "";
        if (1) {
            // Tracking success : Draw the tracked object
            rectangle(frame, rect, Scalar(255, 0, 0), 2, 1);
        } else {
            // Tracking failure detected.
            putText(frame, "Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
        }

        //        // Display tracker type on frame
        //        putText(frame, trackerType + " Tracker", Point(100, 20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
        //
        //        // Display FPS on frame
        //        putText(frame, "FPS : " + SSTR(int(fps)), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
        //video2.open("rtsp://192.168.1.111//user=admin&password=&channel=1&stream=1.sdp?real_stream--rtp-caching=0", cv::CAP_FFMPEG);

        // Display frame.
        imshow("Tracking", frame);

        // Exit if ESC pressed.
        int k = waitKey(1);
        if (k == 27) {
            serial_port.Close();
            break;
        }
    }// END OF INFINITE LOOP OF OBJECT TRACKING

}

void findMotion(VideoCapture& cap, Rect2d& rect) {
    cout << "FINDING MOTION!!! \n";
    Mat flow, frame;

    int rectangleNum;
    // some faster than mat image container
    UMat flowUmat, prevgray;


    Point topLeftAreaPoint;
    Point bottomRightAreaPoint;
    int movingObjectWasDetected = 0;

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
            if (robotCam)flip(img, img, 1);

            resize(img, img, Size(640, 480));


            // save original for later


            img.copyTo(original);
            resize(original, original, Size(640, 480));

            // just make current frame gray


            cvtColor(img, img, COLOR_BGR2GRAY);


            // For all optical flow you need a sequence of images.. Or at least 2 of them. Previous                           //and current frame


            //if there is no current frame


            // go to this part and fill previous frame


            //else {


            // img.copyTo(prevgray);


            //   }


            // if previous frame is not empty.. There is a picture of previous frame. Do some                                  //optical flow alg. 




            if (prevgray.empty() == false) {


                // calculate optical flow 


                calcOpticalFlowFarneback(prevgray, img, flowUmat, 0.4, 1, 12, 2, 8, 1.2, 0);


                // copy Umat container to standard Mat


                flowUmat.copyTo(flow);


                // By y += 5, x += 5 you can specify the grid 


                for (int y = 0; y < original.rows; y += 5) {
                    for (int x = 0; x < original.cols; x += 5) {

                        // 106x6 160x3
                        // get the flow from y, x position * 10 for better visibility

                        const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;

                        // draw line at flow direction, first point is pixel coordinate, second pixels is a flow direction coordinate

                        line(original, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255, 0, 0));

                        // draw initial point

                        circle(original, Point(x, y), 1, Scalar(0, 0, 0), -1);

                    }
                }
                bool needToCheckMotionArea = 1;
                //calculate area motion weight of 18 areas == chose the best one
                vector<int > motionAreaWeights(18, 0);
                vector<Point > topLeftAreaCoordinatesForRect(18, Point(0, 0));
                int numberOf_X_PixelsInOneArea = 106;
                int numberOf_Y_PixelsInOneArea = 160;
                if (needToCheckMotionArea) {
                    for (int i = 0; i < 6; i++) // 6 by 3 == 18 squares
                    {
                        for (int j = 0; j < 3; j++) {
                            for (int x = numberOf_X_PixelsInOneArea * i; x < numberOf_X_PixelsInOneArea * (i + 1); x++) {
                                for (int y = numberOf_Y_PixelsInOneArea * j; y < numberOf_Y_PixelsInOneArea * (j + 1); y++) {
                                    const Point2f flowatxy = flow.at<Point2f>(y, x) * 100;

                                    Point difference(y - (y + flowatxy.y), x - (x + flowatxy.x));
                                    int length = sqrt(difference.x * difference.x + difference.y * difference.y);
                                    //                                    cout << length << "length \n";
                                    //                                    if (i == 5 && j == 1){
                                    //                                    if (i == 0 && j == 0) {
                                    //                                        cout << "pixel xy:" << x << " " << y << " . flow xy is: " << flowatxy.x << " " << flowatxy.y << ". length:" << length << endl;
                                    //
                                    //                                    }
                                    if (length > 1000) motionAreaWeights.at(i * 3 + j) += length;
                                }
                            }
                            topLeftAreaCoordinatesForRect.at(i * 3 + j).x = i;
                            topLeftAreaCoordinatesForRect.at(i * 3 + j).y = j;
                        }
                    }
                    int max = -1;
                    int areaNumWithMaxMotion = -1;
                    max = motionAreaWeights.at(0);
                    //                    for (int i = 0; i < motionAreaWeights.size(); i++) cout << i << " " << motionAreaWeights.at(i) << endl;
                    for (int i = 1; i < motionAreaWeights.size(); i++) {
                        if (motionAreaWeights.at(i) != 0)
                            if (max < motionAreaWeights.at(i)) {
                                max = motionAreaWeights.at(i);
                                areaNumWithMaxMotion = i;
                            }
                    }
                    if (areaNumWithMaxMotion != -1) {
                        movingObjectWasDetected = 1;
                        topLeftAreaPoint = Point(topLeftAreaCoordinatesForRect.at(areaNumWithMaxMotion).x*numberOf_X_PixelsInOneArea, topLeftAreaCoordinatesForRect.at(areaNumWithMaxMotion).y * numberOf_Y_PixelsInOneArea);
                        bottomRightAreaPoint = Point(topLeftAreaCoordinatesForRect.at(areaNumWithMaxMotion).x * numberOf_X_PixelsInOneArea + numberOf_X_PixelsInOneArea, topLeftAreaCoordinatesForRect.at(areaNumWithMaxMotion).y * numberOf_Y_PixelsInOneArea + numberOf_Y_PixelsInOneArea);
                        rect = Rect(topLeftAreaPoint, bottomRightAreaPoint);
                    }
                    rectangleNum = areaNumWithMaxMotion;
                    //                    cout << "max movement was detected in " << areaNumWithMaxMotion << ". Value is: " << max << endl;

                }
                rectangle(original, rect, Scalar(255, 0, 0), 2, 1);
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

        }
    }// END OF INFINITE LOOP OF MOTION DETECTION
}

int main(int argc, const char** argv) {

    // add your file name

    VideoCapture cap(0);
    Rect2d rect;
    findMotion(cap, rect);
    makeTracking(cap, rect);

}

