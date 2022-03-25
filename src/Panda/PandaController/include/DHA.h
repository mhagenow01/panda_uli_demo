#ifndef DHA_INCLUDE_H
#define DHA_INCLUDE_H

#include <Eigen/Core>
#include <iostream>
#include <cmath>
using namespace std;

class DHA {
public:
    double a, alpha, d;
    int n;
    DHA(double a, double d, double alpha, int n = -1)
        : a(a), d(d), alpha(alpha), n(n) {}
    
    Eigen::Matrix4d to_matrix(const std::array<double, 7>& q) {
        double theta = 0;
        if (this->n >= 0 || this->n < q.size()) {
            theta = q[this->n];
        }
        Eigen::MatrixXd trans(4,4);
        // Column major modified DH parameters
        trans << cos(theta), sin(theta) * cos(alpha), sin(theta) * sin(alpha), 0,
                -sin(theta), cos(theta) * cos(alpha), cos(theta) * sin(alpha), 0,
                          0,             -sin(alpha),              cos(alpha), 0,
                          a,         -d * sin(alpha),          d * cos(alpha), 1;
        return trans.transpose();
    }

    Eigen::Matrix4d to_matrix_derivative(const std::array<double, 7>& q) {
        double theta = 0;
        if (this->n >= 0 || this->n < q.size()) {
            theta = q[this->n];
        }
        Eigen::MatrixXd trans(4,4);
        // Column major modified DH parameters
        trans << -sin(theta), cos(theta) * cos(alpha), cos(theta) * sin(alpha), 0,
                 -cos(theta),-sin(theta) * cos(alpha),-sin(theta) * sin(alpha), 0,
                          0,                        0,                       0, 0,
                          0,                        0,                       0, 0;
        return trans.transpose();
    }
};

#endif