#include "Kinematics.h"
#include <array>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include "DHA.h"
#include <iostream>
using namespace std;

namespace PandaController {

    Eigen::Matrix<double, 4, 4> EEFromDHA(array<double, 7> q, vector<DHA> dha, Eigen::Matrix<double, 4, 4> ee_link) {
        // https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters
        
        Eigen::Matrix<double, 4, 4> ee_trans = Eigen::MatrixXd::Identity(4,4);
        for (int i = 0; i < dha.size(); i++) {
            ee_trans = ee_trans * dha[i].to_matrix(q);
        }
        return ee_trans * ee_link;
    }

    array<double, 42> jacobianFromDHA(array<double, 7> q, vector<DHA> dha, Eigen::Matrix<double, 4, 4> ee_link) {
        vector<Eigen::Matrix4d> transforms = vector<Eigen::Matrix4d>();
        vector<Eigen::Matrix4d> transforms_derivative = vector<Eigen::Matrix4d>();
        Eigen::Matrix4d ee_trans = Eigen::MatrixXd::Identity(4,4);
        for (int i = 0; i < dha.size(); i++) {
            auto trans = dha[i].to_matrix(q);
            ee_trans = ee_trans * trans;
            transforms.push_back(trans);
            transforms_derivative.push_back(dha[i].to_matrix_derivative(q));
        }
        auto R = (ee_trans * ee_link).topLeftCorner(3, 3);

        Eigen::Matrix<double, 6, 7> jacobian = Eigen::MatrixXd::Zero(6, 7);
        for (int j = 0; j < 7; j++) {
            Eigen::Matrix4d jac = Eigen::MatrixXd::Identity(4,4);
            for (int i = 0; i < transforms.size(); i++) {
                if (i == j) {
                    jac = jac * transforms_derivative[i];
                } else {
                    jac = jac * transforms[i];
                }
            }
            jac = jac * ee_link;
            jacobian(0, j) = jac(0, 3);
            jacobian(1, j) = jac(1, 3);
            jacobian(2, j) = jac(2, 3);

            auto W = jac.topLeftCorner(3,3) * R.transpose();
            jacobian(3, j) = W(2,1); // w_x
            jacobian(4, j) = W(0,2); // w_y
            jacobian(5, j) = W(1,0); // w_z
        }
        
        array<double, 42> jacobian_array{};
        // Pointer magic
        Eigen::Map<Eigen::MatrixXd>(jacobian_array.data(), 6, 7) = jacobian;
        return jacobian_array;
    }
}