#include "kalman.h"

KalmanFilter2D::KalmanFilter2D()
{
    for (int i = 0; i < 4; ++i)
    {
        x[i] = 0.0;
        for (int j = 0; j < 4; ++j)
        {
            P[i][j] = 0.0;
            Q[i][j] = 0.0;
        }
    }
    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 4; ++c)
            H[r][c] = 0.0;
    H[0][0] = 1.0;
    H[1][1] = 1.0;

    R[0][0] = 25.0; R[0][1] = 0.0;
    R[1][0] = 0.0;  R[1][1] = 25.0;

    for (int i = 0; i < 4; ++i) Q[i][i] = 0.1;
}

void KalmanFilter2D::init(double x0, double y0, double vx0, double vy0)
{
    x[0] = x0; x[1] = y0; x[2] = vx0; x[3] = vy0;

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            P[i][j] = 0.0;

    P[0][0] = 10.0;
    P[1][1] = 10.0;
    P[2][2] = 100.0;
    P[3][3] = 100.0;
}

void KalmanFilter2D::predict(double dt)
{
    // F
    double F[4][4] = {
        {1.0, 0.0, dt,  0.0},
        {0.0, 1.0, 0.0, dt },
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };

    // x = F*x
    double nx[4] = {0,0,0,0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            nx[i] += F[i][j] * x[j];
    for (int i = 0; i < 4; ++i) x[i] = nx[i];

    // P = FPF^T + Q
    double FP[4][4] = {0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                FP[i][j] += F[i][k] * P[k][j];

    double FPFt[4][4] = {0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                FPFt[i][j] += FP[i][k] * F[j][k]; // F^T

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            P[i][j] = FPFt[i][j] + Q[i][j];
}

void KalmanFilter2D::update(double zx, double zy)
{
    double z[2] = {zx, zy};

    // y = z - Hx
    double Hx[2] = {0,0};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 4; ++j)
            Hx[i] += H[i][j] * x[j];

    double yv[2] = { z[0] - Hx[0], z[1] - Hx[1] };

    // S = HPH^T + R
    double HP[2][4] = {0};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                HP[i][j] += H[i][k] * P[k][j];

    double HPHt[2][2] = {0};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 4; ++k)
                HPHt[i][j] += HP[i][k] * H[j][k];

    double S[2][2] = {
        { HPHt[0][0] + R[0][0], HPHt[0][1] + R[0][1] },
        { HPHt[1][0] + R[1][0], HPHt[1][1] + R[1][1] }
    };

    // inv(S)
    double det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    if (std::fabs(det) < 1e-9) return;
    double invDet = 1.0 / det;
    double invS[2][2] = {
        {  S[1][1]*invDet, -S[0][1]*invDet },
        { -S[1][0]*invDet,  S[0][0]*invDet }
    };

    // K = P H^T invS
    double PHt[4][2] = {0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 4; ++k)
                PHt[i][j] += P[i][k] * H[j][k];

    double K[4][2] = {0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
                K[i][j] += PHt[i][k] * invS[k][j];

    // x = x + K y
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 2; ++j)
            x[i] += K[i][j] * yv[j];

    // P = (I - K H) P
    double KH[4][4] = {0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 2; ++k)
                KH[i][j] += K[i][k] * H[k][j];

    double IminusKH[4][4] = {0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            IminusKH[i][j] = (i==j?1.0:0.0) - KH[i][j];

    double newP[4][4] = {0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                newP[i][j] += IminusKH[i][k] * P[k][j];

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            P[i][j] = newP[i][j];
}
