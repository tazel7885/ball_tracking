//
// Created by tazel78885 on 2022/06/13.
//

#include <ball_tracking/Kalman.h>

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::Predict() {
    x = F * x;
    Eigen::MatrixXd Ft = F.transpose();
    P = F * P * Ft + Q;
}

void KalmanFilter::Update(const Eigen::VectorXd &z) {
    Eigen::VectorXd z_pred = H * x;
    Eigen::VectorXd y = z - z_pred;
    Eigen::MatrixXd Ht = H.transpose();
    Eigen::MatrixXd S = H * P * Ht + R;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd PHt = P * Ht;
    Eigen::MatrixXd K = PHt * Si;

    //new estimate
    x = x + (K * y);
    long x_size = x.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    P = (I - K * H) * P;
}