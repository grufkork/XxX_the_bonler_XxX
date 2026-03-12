#ifndef EIGENMODEL_H
#define EIGENMODEL_H

#include <ArduinoEigenDense.h>

using namespace std;
using namespace Eigen;

class StateSpaceModel {
public:
    // System dimensions (default)
    int n = 4;          // Number of states
    int m = 2;          // Number of inputs
    int p = 4;          // Number of outputs
    float Ts = 0.01;    // Sampling time

    // Matrices
    MatrixXf Ac, Bc, C, Ad, Bd, Kf, I;
    MatrixXf Qx, Qu, P, P_prev, P_pred, S;
    MatrixXf K_lqr;

    // Vectors
    VectorXf x, x_prev, x_pred, u_prev, v;
    VectorXf x_ref;

    StateSpaceModel();
    VectorXf kalmanFilter(const VectorXf& y_meas);
    void discretize_state_matricies();
    void solveRicatti();
    void resetKalman();
};

// Global model instance
extern StateSpaceModel Model;

#endif // EIGEN_MODEL_H
