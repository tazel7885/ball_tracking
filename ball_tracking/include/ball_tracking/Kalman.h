//
// Created by tazel78885 on 2022/06/13.
//

#ifndef KALMAN_H_
#define KALMAN_H_

#include <Eigen/Dense>

class KalmanFilter{
public:
    KalmanFilter();

    ~KalmanFilter();

    void Predict();

    void Update(const Eigen::VectorXd &z);

    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    Eigen::MatrixXd F;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;
};

#endif