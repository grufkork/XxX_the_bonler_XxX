#include "eigenModel.h"

// ================= Constructor =================
StateSpaceModel::StateSpaceModel() {
    Ac.resize(n, n);
    Bc.resize(n, n);
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

// ================= Discretization and Ricatti =================
void StateSpaceModel::discretize_state_matricies(){
    // Discretization (Taylor series)
    MatrixXf Psi = MatrixXf::Zero(n,n);
            
    Psi = I*Ts + (Ac*(pow(Ts,2) / 2)) + (Ac*Ac*(pow(Ts,3) / 6)) + (Ac*Ac*Ac*(pow(Ts,4) / 24)) + (Ac*Ac*Ac*Ac*(pow(Ts,5) / 120));
    Ad = I + Ac*Psi;
    Bd = Psi*Bc;
}


void StateSpaceModel::solveRicatti() {
    // Ad and Bd are discretized state space matricies
    // In some cases the Riccati solution might not converge,
    // The LQR gain L will be zero, preventing state based input.

    // Solve Discrete-time Algebraic Riccati Equation (DARE)
    MatrixXf P_tmp = MatrixXf::Zero(n, n);
    float tolerance = 1;
    int max_iterations = 1000;
    int i = 0;

    while ((P - P_tmp).norm() > tolerance && i <= max_iterations) {
        P_tmp = P;

        MatrixXf K = P*Bd*(R + Bd.transpose()*P*Bd).inverse();
        P = Q + Ad.transpose()*(P - K*Bd.transpose()*P)*Ad;

        if (i == max_iterations){
            Serial.println("The Riccati matrix P has not converged!");
        }
        i++;
    };
    // Compute LQR Gain: L = (R + B^T P B)^-1 B^T P A
    K_lqr = (R + Bd.transpose()*P*Bd).inverse()*Bd.transpose()*P*Ad;
}


// ================= Global Instance =================
StateSpaceModel Model;
