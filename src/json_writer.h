#pragma once

#include <string>
#include <map>
#include <vector>

#include "io.h"
#include "cpa.h"

// Serialize results to a JSON file.
void write_json(
    const std::string& path,
    const std::map<std::string, std::vector<Measurement>>& series,
    const std::map<std::string, CpaResult>& final_results,
    const std::map<std::string, Vec2>& final_positions,
    const std::map<std::string, Vec2>& final_velocities,
    const Vec2& own_pos,
    const Vec2& own_vel
);
