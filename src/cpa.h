#pragma once
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Vec2
{
    double x{0.0};
    double y{0.0};
};

struct CpaResult
{
    double cpa_distance{0.0};
    double tcpa{0.0};
    bool collision_risk{false};
    bool closing{false};
    bool valid{false};
};

constexpr double CPA_THRESHOLD_METERS   = 50.0;
constexpr double TCPA_THRESHOLD_SECONDS = 30.0;

inline Vec2 course_to_velocity(double speed, double course_deg)
{
    double rad = course_deg * M_PI / 180.0;
    return Vec2{speed * std::cos(rad), speed * std::sin(rad)};
}

CpaResult compute_cpa(
    const Vec2& own_pos,
    const Vec2& own_vel,
    const Vec2& tgt_pos,
    const Vec2& tgt_vel);
