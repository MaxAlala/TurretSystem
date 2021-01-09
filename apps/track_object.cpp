
#include <opencv2/tracking.hpp>
//#include <opencv2 core="" ocl.hpp="">
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <iostream>

//#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
//https://gist.github.com/guimeira/541e9056364b9491452b7027f12536cc == select rectangle
// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

struct SelectionState {
    Point startPt, endPt, mousePos;
    bool started = false, done = false;

    Rect toRect() {
        return Rect(
                min(this->startPt.x, this->mousePos.x),
                min(this->startPt.y, this->mousePos.y),
                abs(this->startPt.x - this->mousePos.x),
                abs(this->startPt.y - this->mousePos.y));
    }
};

void onMouse(int event, int x, int y, int, void *data) {
    SelectionState *state = (SelectionState*) data;

    switch (event) {
        case EVENT_LBUTTONDOWN:
            state->startPt.x = x;
            state->startPt.y = y;
            state->mousePos.x = x;
            state->mousePos.y = y;
            state->started = true;
            break;

        case EVENT_LBUTTONUP:
            state->endPt.x = x;
            state->endPt.y = y;
            state->done = true;
            break;

        case EVENT_MOUSEMOVE:
            state->mousePos.x = x;
            state->mousePos.y = y;
            break;
    }
}

Rect selectRect(Mat image, Scalar color = Scalar(255, 0, 0), int thickness = 2) {
    const string window = "rect";
    SelectionState state;
    namedWindow(window, WINDOW_NORMAL);
    setMouseCallback(window, onMouse, &state);

    while (!state.done) {
        waitKey(100);

        if (state.started) {
            Mat copy = image.clone();
            Rect selection = state.toRect();
            rectangle(copy, selection, color, thickness);
            imshow(window, copy);
        } else {
            imshow(window, image);
        }
    }

    return state.toRect();
}
//rtsp://192.168.1.222:554/user=admin&password=&channel=1&stream=0.sdp?real_stream

int main(int argc, char **argv) {
    // List of tracker types in OpenCV 3.4.1
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
    // Read video
//    VideoCapture video("rtsp://192.168.1.111//user=admin&password=&channel=1&stream=1.sdp?real_stream--rtp-caching=0");
//    VideoCapture video("rtsp://192.168.1.111//user=admin&password=&channel=1&stream=1.sdp?real_stream--rtp-caching=0");
  
     VideoCapture video(0);

    // Exit if video is not opened
    if (!video.isOpened()) {
        cout << "Could not read video file" << endl;
        return 1;
    }

    // Read first frame 
    Mat frame;

    Size size(640, 480);
    bool ok = video.read(frame);
    flip(frame,frame,1);

    int divider = 1;
    //     Size size(frame.cols/divider, frame.rows/divider);

    //    
    //      while (video.read(frame)) {
    ////               Size size(frame.cols/divider, frame.rows/divider);
    //  resize(frame, frame, size);
    ////        resize(frame, frame, size);
    //        imshow("Tracking", frame);
    //                // Exit if ESC pressed.
    //        int k = waitKey(1);
    //        if (k == 27) {
    //            break;
    //        }
    //      }

    //  resize(frame, frame, size);
    Rect2d rect = selectRect(frame);
    cout << frame.size() << endl;
    // Define initial bounding box 
    //    Rect2d bbox(287, 23, 86, 320); 

    // Uncomment the line below to select a different bounding box 
    // bbox = selectROI(frame, false); 
    // Display bounding box. 
    rectangle(frame, rect, Scalar(255, 0, 0), 2, 1);

    imshow("Tracking", frame);
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
    while (video.read(frame)) {
        flip(frame,frame,1);
        //        resize(frame, frame, size);
        //  video2.release();
        // Start timer
        double timer = (double) getTickCount();

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
                if (AxisWasSlowedDown[0] == 1) {}// if a speed already was slowed down do nothing
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

            if (whichCommandIsRunningNow[0] == -1) {} else {
                serial_port.Write("mca 5 " + to_string(speeds[0]) + " -1\n"); // go up
                cout << "less 5\n";
                whichCommandIsRunningNow[0] = -1;
            }
        // camera view is below an object
        } else if (rect.y + rect.height / 2 > frame.rows / 2 + steps) {
            
            
                        // slow motion when some border was approached
            if (rect.y + rect.height / 2 > frame.rows / 2 + steps && rect.y + rect.height / 2 < frame.rows / 2 + stepsForMotionSlowing) {
                if (AxisWasSlowedDown[0] == 1) {}// if a speed already was slowed down do nothing
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
        ///////////// end Y
        //        cout << serial_port.GetNumberOfBytesAvailable() << "bytes \n";
        if (serial_port.GetNumberOfBytesAvailable() == 0) {
        } else serial_port.ReadLine(str);

        if (str == "") {
        } else cout << str << "\n";

        str = "";
        if (ok) {
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
    }
}
