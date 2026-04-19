#ifndef EIGENMODEL_H
#define EIGENMODEL_H

#include <ArduinoEigenDense.h>

using namespace std;
using namespace Eigen;

class StateSpaceModel {
public:
    // System dimensions (default)
    static const int n = 4;          // Number of states
    static const int m = 2;          // Number of inputs
    static const int p = 4;          // Number of outputs
    float Ts = 0.015;    // Sampling time

    // Matrices
    Matrix<float, n, n> Ac, Ad, Qx, P_prev, P_pred, I, P, Q;
    Matrix<float, n, m> Bc, Bd;
    Matrix<float, m, n> K_lqr;
    Matrix<float, m, m> Qu;
    Matrix<float, p, n> C;
    Matrix<float, n, p> Kf;
    Matrix<float, p, p> S, R;
    

    
    // MatrixXf Ac, Bc, C, Ad, Bd, Kf, I;
    // MatrixXf Qx, Qu, P, P_prev, P_pred, S;
    // MatrixXf Q, R;
    // MatrixXf K_lqr;

    // Vectors
    
    Vector<float, n> x, x_prev, x_pred, x_ref;
    Vector<float, m> u_prev;
    Vector<float, p> v;
    

    // VectorXf x, x_prev, x_pred, u_prev, v;
    // VectorXf x_ref;

    StateSpaceModel();
    Vector<float, n> kalmanFilter(const Vector<float, p>& y_meas);
    void discretize_state_matricies();
    void solveRicatti();
    void resetKalman();
};

// Global model instance
extern StateSpaceModel Model;

#endif // EIGEN_MODEL_H
