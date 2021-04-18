
#include <iostream>
#include "motionDetector.h"
#include "PixelToMotorStepsConverter.h"
#include "Eye.h"
#include "TurretSystem.h"
#include "InverseForwardKinematicsModel.h"
/*
             int x_resolution,
            int y_resolution,
            int x_divider,
            int y_divider,
            bool shouldFlipCam,
            int rectWidth,
            int rectHeight
            );
 */
//bool wasPointChosen = false;
//cv::Point point(0, 0);
//
//void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
//    if (event == EVENT_LBUTTONDOWN) {
//        point.x = x;
//        point.y = y;
//        wasPointChosen = true;
//
//        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
//    }
//    //    } else if (event == EVENT_RBUTTONDOWN) {
//    //        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
//    //    } else if (event == EVENT_MBUTTONDOWN) {
//    //        cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
//    //    } else if (event == EVENT_MOUSEMOVE) {
//    //        cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
//    //    }
//}

int main(int argc, const char** argv) {
    using namespace std;
    
    TurretSystem ts;
//    ts.calibrateItself();
    ts.run();
//    StepperMotorController smc(400, 400, 4, "/dev/ttyACM0");
//    smc.moveAFewSteps(0, 200);
    
    
//    /////////////////////
//    InverseForwardKinematicsModel ifk(3);
//    ifk.doForwardKinematics();
//    Eigen::Vector3d pointWhereToGo{10, 10, 25};
//    ifk.doInverseKinematics(pointWhereToGo);
////    pointWhereToGo = {10,0,25};
////    ifk.doInverseKinematics(pointWhereToGo);
//        pointWhereToGo = {10,-10,25};
//    ifk.doInverseKinematics(pointWhereToGo);
//    
}


