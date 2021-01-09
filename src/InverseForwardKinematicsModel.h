/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   InverseForwardKinematicsModel.h
 * Author: sol
 *
 * Created on January 4, 2021, 9:52 AM
 */

#ifndef INVERSEFORWARDKINEMATICSMODEL_H
#define INVERSEFORWARDKINEMATICSMODEL_H
#include <vector>
#include <Eigen/Dense>

class InverseForwardKinematicsModel {
public:
    InverseForwardKinematicsModel(int axisNumber_);
    InverseForwardKinematicsModel(const InverseForwardKinematicsModel& orig);
    void doForwardKinematics();
    Eigen::Vector2d doInverseKinematics(Eigen::Vector3d v);
    void calculateDH();
    void truncate(double& val, int numDigits);
    void checkIfAlmostEqual(double &x, double &y);
    double fromGradToRad(double d);
    void fromGradToRad(std::vector<double> &vec);
    double fromRadToGrad(double d);
    void destroyPseudoZeros(Eigen::Matrix<double, 3, 3 > &mat);
    Eigen::Vector3d convertCoordinateFromCameraToWorldFrame(Eigen::Vector3d pointWhereToGo);

    virtual ~InverseForwardKinematicsModel();

private:
    int axisNumber;
    std::vector<double > link_length;
    std::vector<double > thetas;
    std::vector<double > default_thetas;
    std::vector<double > offset_d;
    std::vector<double > linkLength_a;
    std::vector<double > alphas;
    std::vector<Eigen::Matrix <double, 4, 4 >> DH;
    Eigen::Matrix<double, 3, 4 > DH_parameters;
    Eigen::Matrix<double, 4, 4 > DH_2_0; // from 2 to 0
    int numOfPixelsInOneCM; 
};

#endif /* INVERSEFORWARDKINEMATICSMODEL_H */

