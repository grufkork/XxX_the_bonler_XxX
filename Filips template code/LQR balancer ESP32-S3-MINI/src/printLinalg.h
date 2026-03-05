#ifndef PRINT_LINALG_H
    #define PRINT_LINALG_H
        #include <ArduinoEigenDense.h>
        using namespace Eigen;
#endif

template <typename Derived>
void printMatrix(const char* name, const Eigen::MatrixBase<Derived>& m) {
    Serial.print(name);
    Serial.print(" (");
    Serial.print(m.rows());
    Serial.print("x");
    Serial.print(m.cols());
    Serial.println("):");

    for (int i = 0; i < m.rows(); ++i) {
        for (int j = 0; j < m.cols(); ++j) {
            Serial.print(m(i, j));
            Serial.print(" ");
        }
        Serial.println();
    }
    Serial.println();
}

template <typename Derived>
void printVector(const char* name, const Eigen::MatrixBase<Derived>& v) {
    Serial.print(name);
    Serial.print(" (");
    Serial.print(v.size());
    Serial.println("):");

    for (int i = 0; i < v.size(); ++i) {
        Serial.println(v(i));
    }
    Serial.println();
}

