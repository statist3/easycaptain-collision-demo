#pragma once
#include <vector>
#include <string>
#include <map>
#include "cpa.h"

void print_ascii_radar(
    const std::vector<std::pair<std::string, Vec2>>& positions,
    const std::map<std::string, CpaResult>& results,
    const Vec2& own_pos);
