// 6dofArmSoftware.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
//#include "Matrix.h"
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#define M_PI           3.14159265358979323846  /* pi */
#include <sstream>
#include <string>

//28.28427125 0 38.28427125
std::vector<double > givenPosition = {21.21320344, 0, 31.21320344};

void fromGradToRad(std::vector<double> &vec) {
    std::for_each(vec.begin(), vec.end(), [](double &d) {
        d = d / 180 * M_PI; });

}

void truncate(double& val, int numDigits) {
    using namespace std;
    std::string output = std::to_string(val).substr(0, numDigits + 1);
    if (output.find('.') == string::npos ||
            output.back() == '.') {
        output.pop_back();
    }
    std::stringstream ss;
    ss << output;
    ss >> val;
}

double fromGradToRad(double d) {
    return d / 180 * M_PI;
}

double fromRadToGrad(double d) {
    return d * 180 / M_PI;
}

void destroyPseudoZeros(Eigen::Matrix<double, 3, 3 > &mat) {
    for (int i = 0; i < mat.rows(); i++)
        for (int j = 0; j < mat.cols(); j++)
            if (mat(i, j) < 0.00001)
                if (mat(i, j) > -0.00001)
                    mat(i, j) = 0.0;
}

std::vector<Eigen::Matrix <double, 4, 4 >> DH(6);
Eigen::Matrix<double, 6, 4 > DH_parameters;

void calculateDH() {
    if (DH.size() > 0)DH.clear();
    //std::cout << "hey~";
    for (int i = 0; i < 6; i++) {
        Eigen::Matrix<double, 4, 4> current_from_i_to_i_plus_one_mat;
        current_from_i_to_i_plus_one_mat << cos(DH_parameters(i, 0)), -sin(DH_parameters(i, 0)) * cos(DH_parameters(i, 3)), sin(DH_parameters(i, 0)) * sin(DH_parameters(i, 3)), DH_parameters(i, 2) * cos(DH_parameters(i, 0)),
                sin(DH_parameters(i, 0)), cos(DH_parameters(i, 0)) * cos(DH_parameters(i, 3)), -cos(DH_parameters(i, 0)) * sin(DH_parameters(i, 3)), DH_parameters(i, 2) * sin(DH_parameters(i, 0)),
                0, sin(DH_parameters(i, 3)), cos(DH_parameters(i, 3)), DH_parameters(i, 1),
                0, 0, 0, 1;
        DH.push_back(Eigen::Matrix<double, 4, 4>(current_from_i_to_i_plus_one_mat));

    }
    //    std::cout << "hey~2";
}
Eigen::Matrix<double, 3, 3 > checkR3_4;
Eigen::Matrix<double, 3, 3 > checkR4_5;
Eigen::Matrix<double, 3, 3 > checkR5_6;
Eigen::Matrix<double, 3, 3 > checkR6_3;

std::vector<double > inverse_thetas(6);

void calculateCheckR6_3() {
    Eigen::Matrix<double, 3, 3 > r34;
    r34 <<
            cos(inverse_thetas[3]), 0, sin(inverse_thetas[3]),
            sin(inverse_thetas[3]), 0, -cos(inverse_thetas[3]),
            0, 1, 0;
    checkR3_4 = r34;
    Eigen::Matrix<double, 3, 3 > r45;
    r45 <<
            cos(inverse_thetas[4]), 0, -sin(inverse_thetas[4]),
            sin(inverse_thetas[4]), 0, cos(inverse_thetas[4]),
            0, -1, 0;
    checkR4_5 = r45;
    Eigen::Matrix<double, 3, 3 > r56;
    r56 <<
            cos(inverse_thetas[5]), -sin(inverse_thetas[5]), 0,
            sin(inverse_thetas[5]), cos(inverse_thetas[5]), 0,
            0, 0, 1;
    //    Eigen::Matrix<double, 3, 3 > r36;
    checkR6_3 = checkR3_4 * checkR4_5 * checkR5_6;
    destroyPseudoZeros(checkR6_3);
}

bool checkIfEqual(double &x, double &y) {
    std::cout.precision(17);
    std::cout << "checkIfEqual:  \n";
    std::cout << "x: " << x << ", y: " << y << std::endl;
    std::cout << "abs(x - y): " << abs(abs(x) - abs(y)) << std::endl;
    if (x - y == 0) return true;
    if (abs(abs(x) - abs(y)) < 0.000001) {
        bool isDifferentSignes = false; // false == same signes
        if (x >= 0 && y <= 0) {
            isDifferentSignes = true;
        }

        if (y >= 0 && x <= 0) {
            isDifferentSignes = true;
        }

        if (isDifferentSignes == false) {
            x = y;
            return true;
        }

        return false;
    }
}

bool checkIfEqual(Eigen::Matrix<double, 3, 3 > &checkR6_3, Eigen::Matrix<double, 3, 3 > &R6_3) {
    bool equal = true;

    for (int j = 0; j < 3; j++)
        for (int i = 0; i < 3; i++)
            if (checkIfEqual(checkR6_3(i, j), R6_3(i, j))) {
            } else {
                equal = false;
            }

    if (equal) std::cout << "Matrices are equal! \n";
    else
        std::cout << "Matrices are not equal! \n";
    return equal;
}

void checkIfAlmostEqual(double &x, double &y) {
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

int main() {
    //    Matrix<double > mexpample = {
    //        {1, 2, 3},
    //        {4, 7, 6},
    //        {7, 8, 9}
    //    };

    //    std::cout << mexpample.Det() << " determinant \n";
    using namespace std;
    int link_length [] = {10, 10, 10, 10, 10};
    std::cout << "Hello World!\n";
    vector<double > thetas = {-0, -90, -90, 0, 90, 0};
    // 6dof version
    //    vector<double > thetas = {-0, -90, -90, 0, 0, 180};
    vector<double > offset_d = {link_length[0], 0, 0, link_length[2] + link_length[3], 0, 0};
    //6dof version
    //vector<double > offset_d = {link_length[0], 0, 0, link_length[2]+link_length[3], 0, link_length[4]};

    vector<double > linkLength_a = {0, link_length[1], 0, 0, link_length[4], 0};
    //6dof version
    //    vector<double > linkLength_a = {0, link_length[1], 0, 0, 0, 0};

    vector<double > alphas = {-90, 180, -90, 90, 0, 0};
    //6dof version
    //    vector<double > alphas = {-90, 180, -90, 90, -90, 0};

    /*vector<Matrix <double >> DH;*/
    fromGradToRad(thetas);
    fromGradToRad(alphas);

    DH_parameters(0, 0) = thetas.at(0);
    DH_parameters(1, 0) = thetas.at(1);
    DH_parameters(2, 0) = thetas.at(2);
    DH_parameters(3, 0) = thetas.at(3);
    DH_parameters(4, 0) = thetas.at(4);
    DH_parameters(5, 0) = thetas.at(5);

    DH_parameters(0, 1) = offset_d.at(0);
    DH_parameters(1, 1);
    DH_parameters(2, 1);
    DH_parameters(3, 1) = offset_d.at(3);
    DH_parameters(4, 1);
    DH_parameters(5, 1);

    DH_parameters(0, 2);
    DH_parameters(1, 2) = linkLength_a.at(1);
    DH_parameters(2, 2);
    DH_parameters(3, 2);
    DH_parameters(4, 2) = linkLength_a.at(4);
    DH_parameters(5, 2);

    DH_parameters(0, 3) = alphas.at(0);
    DH_parameters(1, 3) = alphas.at(1);
    DH_parameters(2, 3) = alphas.at(2);
    DH_parameters(3, 3) = alphas.at(3);
    DH_parameters(4, 3) = alphas.at(4);
    DH_parameters(5, 3) = alphas.at(5);
    cout << DH_parameters << endl;
    calculateDH();
    //for (int i = 0; i < 6; i++)
    //{
    //	DH.push_back(Matrix<double >{
    //		{ cos(DH_parameters[i][0]), -sin(DH_parameters[i][0]) * cos(DH_parameters[i][3]), sin(DH_parameters[i][0]) * sin(DH_parameters[i][3]), DH_parameters[i][2] * cos(DH_parameters[i][0]) },
    //		{ sin(DH_parameters[i][0]), cos(DH_parameters[i][0]) * cos(DH_parameters[i][3]), -cos(DH_parameters[i][0]) * sin(DH_parameters[i][3]), DH_parameters[i][2] * sin(DH_parameters[i][0]) },
    //		{ 0, sin(DH_parameters[i][3]), cos(DH_parameters[i][3]), DH_parameters[i][1] },
    //		{ 0, 0, 0, 1 }
    //	}
    //	);
    //}
    //std::cout << "hey3~";
    for (int i = 0; i < 6; i++)
        cout << "from " << i << "to" << i + 1 << "== \n" << DH[i] << endl;

    Eigen::Matrix<double, 4, 4 > DH_6_0; // from 6 to 0
    Eigen::Matrix<double, 4, 4 > DH_4_0; // from 6 to 0
    Eigen::Matrix<double, 4, 4 > DH_5_0; // from 6 to 0


    DH_6_0 = DH[0] * DH[1] * DH[2] * DH[3] * DH[4] * DH[5];
    //DH_6_0 = DH[0] * DH[1] * DH[2] * DH[3];


    cout << " DH_6_0 == \n " << DH_6_0 << endl;



    // inverse kinematics

    // calculate theta 1 from top view scheme
    inverse_thetas[0] = atan2(givenPosition[1], givenPosition[0]);
    cout << "atan2 0 and 28 == " << atan2(givenPosition[1], givenPosition[0]) << "\n";
    // calculate theta 2 & 3 from side view
    double thi1;
    double thi2;
    double thi3;
    double r1 = sqrt(givenPosition[0] * givenPosition[0] + givenPosition[1] * givenPosition[1]);
    double r2 = givenPosition[2] - link_length[0];
    double r3 = sqrt(r1 * r1 + r2 * r2);
    cout << r3 << "== r3 before \n";
    truncate(r3, 7);
    cout << r3 << "== r3 after \n";

    cout << r2 << "== r2 before \n";
    truncate(r2, 7);
    cout << r2 << "== r2 after \n";

    cout << r1 << "== r1 before \n";
    truncate(r1, 7);
    cout << r1 << "== r1 after \n";

    thi2 = atan2(r2, r1);
    thi1 = acos(((link_length[2] + link_length[3]) * (link_length[2] + link_length[3]) - link_length[1] * link_length[1] - r3 * r3) / (-2.0 * link_length[1] * r3));
    thi3 = acos((r3 * r3 - (link_length[2] + link_length[3]) * (link_length[2] + link_length[3]) - link_length[1] * link_length[1]) / (-2.0 * link_length[1] * offset_d.at(3)));
    inverse_thetas[2] = fromGradToRad(180) - thi3;
    inverse_thetas[1] = thi2 - thi1;

    DH_parameters(0, 0) += inverse_thetas.at(0);
    DH_parameters(1, 0) += inverse_thetas.at(1);
    DH_parameters(2, 0) += -inverse_thetas.at(2);

    calculateDH();
    Eigen::Matrix<double, 4, 4 > DH_3_0; // from 6 to 0
    DH_3_0 = DH[0] * DH[1] * DH[2];
    DH_6_0 = DH[0] * DH[1] * DH[2] * DH[3] * DH[4] * DH[5];
    DH_4_0 = DH[0] * DH[1] * DH[2] * DH[3];
    DH_5_0 = DH[0] * DH[1] * DH[2] * DH[3] * DH[4];
    cout << "DH_6_0 = \n" << DH_6_0 << endl;
    cout << "DH_5_0 = \n" << DH_5_0 << endl;
    cout << "DH_4_0 = \n" << DH_4_0 << endl;
    Eigen::Matrix<double, 3, 3 > R3_0;
    R3_0 <<
            DH_3_0(0, 0), DH_3_0(0, 1), DH_3_0(0, 2),
            DH_3_0(1, 0), DH_3_0(1, 1), DH_3_0(1, 2),
            DH_3_0(2, 0), DH_3_0(2, 1), DH_3_0(2, 2);


    //Matrix<double > R6_0 = {
    //{DH_6_0[0][0], DH_6_0[0][1], DH_6_0[0][2]},
    //{DH_6_0[1][0], DH_6_0[1][1], DH_6_0[1][2]},
    //{DH_6_0[2][0], DH_6_0[2][1], DH_6_0[2][2]}
    //};
    // desired position
    //	Matrix<double > R6_0 = {
    //{-1, 0, 0},
    //{0, -1, 0},
    //{0, 0, 1}
    //	};
    //	Matrix<double > R6_0 = {
    //{1, 0, 0},
    //{0, 1, 0},
    //{0, 0, 1}
    //	};
//    Eigen::Matrix<double, 3, 3 > R6_0;
//    R6_0 <<
//            -1, 0, 0,
//            0, 1, 0,
//            0, 0, -1;
//
//    //	Matrix<double > R6_0 = {
//    //{0, 1, 0},
//    //{-1, 0, 0},
//    //{0, 0, 1}
//    //	};
//
//    Eigen::Matrix<double, 3, 3 > R3_0_inverse(R3_0.inverse());
//    Eigen::Matrix<double, 3, 3 > R6_3(R3_0_inverse * R6_0);
//
//    destroyPseudoZeros(R6_3);
//    std::cout.precision(17);
//    cout << "R6_3: " << R6_3 << endl;
//
//    cout << "inverse R3_0: " << R3_0_inverse << endl;
//    cout << " R3_0: " << R3_0 << endl;
//    cout << "r1, r2, r3: " << r1 << " " << r2 << " " << r3 << " \n";
//    cout << "thi1, thi2, thi3: " << thi1 << " " << thi2 << " " << thi3 << " \n";
//    cout << "thi1, thi2, thi3: " << fromRadToGrad(thi1) << " " << fromRadToGrad(thi2) << " " << fromRadToGrad(thi3) << " \n";
//
//
//    cout << "inversed vals: \n";
//
//    for (int i = 0; i < 3; i++)
//        cout << inverse_thetas[i] << " ";
//    cout << '\n';
//    cout << "fromRadToGrad: \n";
//    for (int i = 0; i < 3; i++)
//        cout << fromRadToGrad(inverse_thetas[i]) << " ";
//
//
//
//
//    vector<int > potentialInvalidValue;
//    //2.33882
//    inverse_thetas[4] = acos(R6_3(2, 2)); //0
//    cout << " \n invtheta 4: " << acos(R6_3(2, 2)) << endl;
//    if (R6_3(2, 2) == 0) potentialInvalidValue.push_back(4);
//    if (-sin(inverse_thetas[4] == 0)) {
//
//        double pseudo_result = 0.000001;
//        inverse_thetas[5] = acos(R6_3(2, 0) / -pseudo_result);
//        if (R6_3(2, 0) / -pseudo_result == 0) potentialInvalidValue.push_back(5);
//        //inverse_thetas[3] = asin(R6_3[1][2] / pseudo_result);
//        inverse_thetas[3] = -acos(-R6_3(0, 2) / pseudo_result);
//        if (-R6_3(0, 2) / pseudo_result == 0) potentialInvalidValue.push_back(3);
//
//    } else {
//        double R6_3_2_0 = R6_3(2, 0);
//        double sin_inv_th4 = sin(inverse_thetas[4]);
//        checkIfAlmostEqual(R6_3_2_0, sin_inv_th4);
//        //-0.719288 / 0.719288
//        inverse_thetas[5] = acos(R6_3_2_0 / (sin_inv_th4));
//
//
//        if (R6_3(2, 0) / (-sin(inverse_thetas[4])) == 0) potentialInvalidValue.push_back(5);
//
//        //inverse_thetas[3] = asin(R6_3[1][2] / (-sin(inverse_thetas[4])));
//        double R6_3_0_2 = R6_3(0, 2);
//        checkIfAlmostEqual(R6_3_2_0, sin_inv_th4);
//
//        inverse_thetas[3] = acos(-R6_3_0_2 / sin_inv_th4);
//        if (-R6_3(0, 2) / sin(inverse_thetas[4]) == 0) potentialInvalidValue.push_back(3);
//    }
//
//
//    //if(acos)
//
//    Eigen::Matrix<double, 3, 3 > sampleR;
//    sampleR <<
//            cos(fromGradToRad(45)), -sin(fromGradToRad(45)), 0,
//            sin(fromGradToRad(45)), cos(fromGradToRad(45)), 0,
//            0, 0, 1;
//
//    //cout << "\n sample R: " << sampleR;
//    //inverse_thetas[5] =
//    calculateCheckR6_3();
//    cout << "\n check R6_3: " << checkR6_3 << endl;
//
//    if (checkIfEqual(checkR6_3, R6_3))
//        cout << "YES! \n" << endl;
    //
    //    //while (true)
    //    if (checkIfEqual(checkR6_3, R6_3)) {
    //        cout << "YES! \n";
    //    } else {
    //        cout << "bad news, checkR6_3 and R6_3 not equal \n";
    //        vector<double> saveInverse_thetas = inverse_thetas;
    //        if (potentialInvalidValue.size() == 1) {
    //            cout << "here1 in first \n";
    //            cout << "change first sign \n";
    //            inverse_thetas[potentialInvalidValue[0]] = -inverse_thetas[potentialInvalidValue[0]];
    //            calculateCheckR6_3();
    //            if (checkIfEqual(checkR6_3, R6_3))cout << "YES! \n";
    //        }
    //        if (potentialInvalidValue.size() == 2) {
    //            cout << "here in second \n";
    //            cout << "change first sign \n";
    //            inverse_thetas[potentialInvalidValue[0]] = -inverse_thetas[potentialInvalidValue[0]];
    //            calculateCheckR6_3();
    //            if (checkIfEqual(checkR6_3, R6_3))cout << "YES! \n";
    //            cout << "change 1 and 2 signs \n";
    //            inverse_thetas[potentialInvalidValue[1]] = -inverse_thetas[potentialInvalidValue[1]];
    //            calculateCheckR6_3();
    //            if (checkIfEqual(checkR6_3, R6_3)) cout << "YES! \n";
    //            inverse_thetas = saveInverse_thetas;
    //            cout << " change 2 sign \n";
    //            inverse_thetas[potentialInvalidValue[1]] = -inverse_thetas[potentialInvalidValue[1]];
    //            calculateCheckR6_3();
    //            if (checkIfEqual(checkR6_3, R6_3)) cout << "YES! \n";
    //
    //
    //        }
    //        if (potentialInvalidValue.size() == 3) {
    //            cout << "here in third \n";
    //            cout << "change 1 sign \n";
    //            inverse_thetas[potentialInvalidValue[0]] = -inverse_thetas[potentialInvalidValue[0]];
    //            calculateCheckR6_3();
    //            if (checkIfEqual(checkR6_3, R6_3))cout << "YES! \n";
    //            cout << "change 1 and 2 signs \n";
    //            inverse_thetas[potentialInvalidValue[1]] = -inverse_thetas[potentialInvalidValue[1]];
    //            calculateCheckR6_3();
    //            if (checkIfEqual(checkR6_3, R6_3)) cout << "YES! \n";
    //            cout << " change all three signs \n";
    //            inverse_thetas[potentialInvalidValue[2]] = -inverse_thetas[potentialInvalidValue[2]];
    //            calculateCheckR6_3();
    //            if (checkIfEqual(checkR6_3, R6_3)) cout << "YES! \n";
    //            ////////////////////////////////////
    //            inverse_thetas = saveInverse_thetas;
    //            cout << " change 2 sign \n";
    //            inverse_thetas[potentialInvalidValue[1]] = -inverse_thetas[potentialInvalidValue[1]];
    //            calculateCheckR6_3();
    //            if (checkIfEqual(checkR6_3, R6_3)) cout << "YES! \n";
    //            cout << " change 2 and 3 signs \n";
    //            inverse_thetas[potentialInvalidValue[2]] = -inverse_thetas[potentialInvalidValue[2]];
    //            calculateCheckR6_3();
    //            if (checkIfEqual(checkR6_3, R6_3)) cout << "YES! \n";
    //            ////////////////////////////////////
    //            inverse_thetas = saveInverse_thetas;
    //            cout << " change 3 sign \n";
    //            inverse_thetas[potentialInvalidValue[2]] = -inverse_thetas[potentialInvalidValue[2]];
    //            calculateCheckR6_3();
    //            if (checkIfEqual(checkR6_3, R6_3)) cout << "YES! \n";
    //            cout << " change 3 and 1 signs \n";
    //            inverse_thetas[potentialInvalidValue[0]] = -inverse_thetas[potentialInvalidValue[0]];
    //            calculateCheckR6_3();
    //            if (checkIfEqual(checkR6_3, R6_3)) cout << "YES! \n";
    //        }
    //
    //        cout << "\n	 R6_3: " << R6_3;
    //        cout << "\n new check R6_3: " << checkR6_3;
    //    }
    //    cout << endl;
    //    for (int i = 3; i < 6; i++)
    //        cout << i + 1 << " theta: " << fromRadToGrad(inverse_thetas[i]) << ", ";

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
