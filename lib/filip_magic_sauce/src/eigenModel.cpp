#include "eigenModel.h"

// ================= Constructor =================
StateSpaceModel::StateSpaceModel() {
    C.resize(p, n);
    Ad.resize(n, n);
    Bd.resize(n, m);
    Kf.resize(n, p);
    I = MatrixXf::Identity(n, n);

    Q.resize(n, n);
    R.resize(p, p);
    P.resize(n, n);
    P_prev.resize(n, n);
    P_pred.resize(n, n);
    S.resize(p, p);

    x.resize(n);
    x_prev.resize(n);
    x_pred.resize(n);
    u_prev.resize(m);
    v.resize(p);

    x_ref.resize(n);
    K_lqr.resize(m, n);

    C = MatrixXf::Identity(p, n);       // measurement matrix
    P_prev = 0.01*MatrixXf::Identity(n, n);  // initial covariance
    P = P_prev;
    x_prev = VectorXf::Zero(n);         // initial state estimate
    x_pred = VectorXf::Zero(n);
    u_prev = VectorXf::Zero(m);
    v = VectorXf::Zero(p);
    x_ref = VectorXf::Zero(n);
}

// ================= Kalman Filter =================
VectorXf StateSpaceModel::kalmanFilter(const VectorXf& y_meas) {
    // Prediction
    x_pred = Ad * x_prev + Bd * u_prev;
    P_pred = Ad * P_prev * Ad.transpose() + Q;

    // Measurement update
    S = C * P_pred * C.transpose() + R;
    Kf = P_pred * C.transpose() * S.inverse();
    v = y_meas - C * x_pred;

    // State update
    x = x_pred + Kf * v;
    P = P_pred - Kf * S * Kf.transpose();

    // Prepare for next iteration
    x_prev = x;
    P_prev = P;

    return x;
}

// ================= Reset Kalman Filter =================
void StateSpaceModel::resetKalman() {
    x_prev = VectorXf::Zero(n);
    x_pred = VectorXf::Zero(n);
    x      = VectorXf::Zero(n);

    P_prev = MatrixXf::Identity(n, n);
    P_pred = P_prev;
    P      = P_prev;

    u_prev = VectorXf::Zero(m);
    v      = VectorXf::Zero(p);
}

// ================= Global Instance =================
StateSpaceModel Model;
