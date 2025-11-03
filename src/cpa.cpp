#include "cpa.h"

CpaResult compute_cpa(
    const Vec2& own_pos,
    const Vec2& own_vel,
    const Vec2& tgt_pos,
    const Vec2& tgt_vel)
{
    double rx = tgt_pos.x - own_pos.x;
    double ry = tgt_pos.y - own_pos.y;

    double vx_rel = tgt_vel.x - own_vel.x;
    double vy_rel = tgt_vel.y - own_vel.y;

    double v2  = vx_rel*vx_rel + vy_rel*vy_rel;
    const double eps = 1e-9;

    if (v2 < eps)
    {
        double dist = std::hypot(rx, ry);
        return CpaResult{ dist, 0.0, false, false, false };
    }

    double dot  = rx*vx_rel + ry*vy_rel;
    double tcpa = -dot / v2;

    double rx_cpa = rx + vx_rel * tcpa;
    double ry_cpa = ry + vy_rel * tcpa;
    double cpa_dist = std::hypot(rx_cpa, ry_cpa);

    bool closing = (tcpa >= 0.0);
    bool risk = closing && (cpa_dist < CPA_THRESHOLD_METERS) && (tcpa < TCPA_THRESHOLD_SECONDS);

    return CpaResult{ cpa_dist, tcpa, risk, closing, true };
}
