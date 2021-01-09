/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   InverseForwardKinematicsModel.cpp
 * Author: sol
 * 
 * Created on January 4, 2021, 9:52 AM
 */

#include "InverseForwardKinematicsModel.h"
#include <string>
#include <iostream>
#include <sstream>
#include <cmath>

#define M_PI           3.1415  /* pi */
// init position == directed up
InverseForwardKinematicsModel::InverseForwardKinematicsModel(int axisNumber_) :
axisNumber{axisNumber_},
thetas{180, -90, 90},
default_thetas{180, 0, 90},
offset_d{link_length[0], 0, link_length[2]},
linkLength_a{0, 0, 0},
alphas{-90, 90, 0},
link_length{25, 0, 5},
DH{axisNumber_},
numOfPixelsInOneCM{1}
{

    fromGradToRad(thetas);
    fromGradToRad(alphas);
    //    thetas[1] += fromGradToRad(90);
    for (int i = 0; i < axisNumber_; i++) {
        DH_parameters(i, 0) = thetas.at(i);
        DH_parameters(i, 1) = offset_d.at(i)*numOfPixelsInOneCM;
        DH_parameters(i, 2) = linkLength_a.at(i)*numOfPixelsInOneCM;
        DH_parameters(i, 3) = alphas.at(i);
    }
}

InverseForwardKinematicsModel::InverseForwardKinematicsModel(const InverseForwardKinematicsModel& orig) {
}

InverseForwardKinematicsModel::~InverseForwardKinematicsModel() {
}

void InverseForwardKinematicsModel::destroyPseudoZeros(Eigen::Matrix<double, 3, 3 > &mat) {
    for (int i = 0; i < mat.rows(); i++)
        for (int j = 0; j < mat.cols(); j++)
            if (mat(i, j) < 0.00001)
                if (mat(i, j) > -0.00001)
                    mat(i, j) = 0.0;
}

void InverseForwardKinematicsModel::truncate(double& val, int numDigits) {
    using namespace std;
    std::string output = std::to_string(val).substr(0, numDigits + 1);

    // case where . at the beginning or at the end of the game
    if (output.find('.') == string::npos ||
            output.back() == '.') {
        output.pop_back();
    }
    std::stringstream ss;
    ss << output;
    ss >> val;
}

void InverseForwardKinematicsModel::checkIfAlmostEqual(double &x, double &y) {
    std::cout.precision(17);
    std::cout << "x: " << x << ", y: " << y << std::endl;
    std::cout << "abs(x - y): " << abs(abs(x) - abs(y)) << std::endl;
    if (abs(abs(x) - abs(y)) < 0.000001) {
        bool isDifferentSignes = false; // false == same signes
        if (x >= 0 && y <= 0) {
            x = -y;
            isDifferentSignes = true;
        }
        if (y >= 0 && x <= 0) {
            y = -x;
            isDifferentSignes = true;
        }
        std::cout << "Yes! checkIfAlmostEqual:  \n";

    }

}

double InverseForwardKinematicsModel::fromGradToRad(double d) {
    return d / 180.0 * M_PI;
}

double InverseForwardKinematicsModel::fromRadToGrad(double d) {
    return d * 180.0 / M_PI;
}

void InverseForwardKinematicsModel::fromGradToRad(std::vector<double> &vec) {
    std::for_each(vec.begin(), vec.end(), [](double &d) {
        d = d / 180 * M_PI; });

}

void InverseForwardKinematicsModel::calculateDH() {
    for (int i = 0; i < axisNumber - 1; i++) {
        DH_parameters(i, 0) = thetas.at(i);
        DH_parameters(i, 1) = offset_d.at(i)*numOfPixelsInOneCM;
        DH_parameters(i, 2) = linkLength_a.at(i)*numOfPixelsInOneCM;
        DH_parameters(i, 3) = alphas.at(i);
    }
    if (DH.size() > 0)DH.clear();
    //std::cout << "hey~";
    for (int i = 0; i < axisNumber - 1; i++) {
        Eigen::Matrix<double, 4, 4> current_from_i_to_i_plus_one_mat;
        current_from_i_to_i_plus_one_mat << cos(DH_parameters(i, 0)), -sin(DH_parameters(i, 0)) * cos(DH_parameters(i, 3)), sin(DH_parameters(i, 0)) * sin(DH_parameters(i, 3)), DH_parameters(i, 2) * cos(DH_parameters(i, 0)),
                sin(DH_parameters(i, 0)), cos(DH_parameters(i, 0)) * cos(DH_parameters(i, 3)), -cos(DH_parameters(i, 0)) * sin(DH_parameters(i, 3)), DH_parameters(i, 2) * sin(DH_parameters(i, 0)),
                0, sin(DH_parameters(i, 3)), cos(DH_parameters(i, 3)), DH_parameters(i, 1),
                0, 0, 0, 1;
        DH.push_back(Eigen::Matrix<double, 4, 4>(current_from_i_to_i_plus_one_mat));

    }
    Eigen::Matrix<double, 4, 4> DH3_2;

    DH3_2 <<
            0, -1, 0, 11*numOfPixelsInOneCM,
            1, 0, 0, 0,
            0, 0, 1, 7*numOfPixelsInOneCM,
            0, 0, 0, 1;
    DH.push_back(DH3_2);

}

void InverseForwardKinematicsModel::doForwardKinematics() {
//    thetas[0] += fromGradToRad(-14.7);
//    thetas[1] += fromGradToRad(6.25);
    calculateDH();
    DH_2_0 = DH[0] * DH[1];
    std::cout << "DH1_0 \n" << DH[0] << "\n";

    std::cout << "DH2_1 \n" << DH[1] << "\n";

    std::cout << "DH3_2 \n" << DH[2] << "\n";


    Eigen::Matrix<double, 4, 4 > DH_3_0; // from 2 to 0
    DH_3_0 = DH[0] * DH[1] * DH[2];
    //    std::cout << " DH_1_0 == \n " << DH[0] << std::endl;
    //    std::cout << " DH_2_1 == \n " << DH[1] << std::endl;
    //    std::cout << " DH_3_2 == \n " << DH[2] << std::endl;
    //    std::cout << " DH_2_0 == \n " << DH_2_0 << std::endl;
    std::cout << " DH_3_0 == \n " << DH_3_0 << std::endl;
}

Eigen::Vector3d InverseForwardKinematicsModel::convertCoordinateFromCameraToWorldFrame(Eigen::Vector3d pointWhereToGo) {

    Eigen::Vector4d pointInCam(pointWhereToGo[0], pointWhereToGo[1], pointWhereToGo[2], 1);
    Eigen::Vector4d pointInWorld;
    calculateDH();
    pointInWorld = DH[0] * DH[1] * DH[2] * pointInCam;

    std::cout << "point in cam frame: " << pointInCam << "\n";
    std::cout << "converted point from cam to world: " << pointInWorld << "\n";
    return Eigen::Vector3d(pointInWorld[0], pointInWorld[1], pointInWorld[2]);
}

Eigen::Vector2d InverseForwardKinematicsModel::doInverseKinematics(Eigen::Vector3d pointWhereToGo) {
    //    Eigen::Vector3d pointWhereToGo{10, 10, 25};
    double r1 = sqrt(pointWhereToGo[0] * pointWhereToGo[0] + pointWhereToGo[1] * pointWhereToGo[1]);
    double r2 = pointWhereToGo[2] - link_length[0];

    std::cout << "where to go:" << pointWhereToGo[0] << " " << pointWhereToGo[1] << " " << pointWhereToGo[2] << "\n";

    std::cout << "r1 and r2: " << r1 << " " << r2 << "\n";
    std::cout << "atan2(r2, r1) in grad ==" << fromRadToGrad(std::atan2(r2, r1)) << "\n";
    std::cout << "atan2(r2, r1)) " << std::atan2(r2, r1) << "\n";

    std::vector<double> calculatedTheta(axisNumber - 1);
    calculatedTheta[1] = -(90 - fromRadToGrad(std::atan2(r2, r1)));
    calculatedTheta[0] = fromRadToGrad(std::atan2(pointWhereToGo[1], pointWhereToGo[0]));
    std::cout << calculatedTheta[0] << " " << calculatedTheta[1] << std::endl;
    Eigen::Vector2d anglesToGo;
    for (int i = 0; i < axisNumber - 1; i++) {
        // calculatedTheta == absolute angle which goes from beginning position
        // 1 thetas ANGLE TO DESTROY CURRENT position + NEW ANGLE calculatedTheta + DEFAULT ANGLE TO DESTROY DEFAULT PART FROM thetas
        anglesToGo[i] = -fromRadToGrad(thetas[i]) + calculatedTheta[i] + default_thetas[i];
        std::cout << i << "axis. Angle to go == " << anglesToGo[i] << "\n";
        thetas[i] = fromGradToRad(calculatedTheta[i] + default_thetas[i]); // or we could just assign calculatedTheta
        //        }
        //        
        //        if ((thetas[i] >= 0 && calculatedTheta[i] <= 0) || (thetas[i] <= 0 && calculatedTheta[i] >= 0)) {
        //            
        //            angleToGo = -thetas[i] + calculatedTheta[i];
        //            thetas[i] = calculatedTheta[i];
        //        }

    }

    doForwardKinematics();
    return anglesToGo;
}






