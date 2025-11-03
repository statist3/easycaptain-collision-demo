#pragma once
#include <cmath>

// ref https://github.com/RahmadSadli/2-D-Kalman-Filter/blob/master/KalmanFilter.py

struct KalmanFilter2D
{
    // state x = [x, y, vx, vy]^T
    double x[4];
    // covariance 4x4
    double P[4][4];
    // measurement H (2x4): observe position only
    double H[2][4];
    // measurement noise R (2x2)
    double R[2][2];
    // process noise Q (4x4)
    double Q[4][4];

    KalmanFilter2D();

    void init(double x0, double y0, double vx0, double vy0);
    void predict(double dt);
    void update(double zx, double zy);

    double getX() const { return x[0]; }
    double getY() const { return x[1]; }
    double getVx() const { return x[2]; }
    double getVy() const { return x[3]; }
};
