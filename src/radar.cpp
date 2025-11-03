#include "radar.h"
#include <iostream>
#include <algorithm>
#include <cmath>

void print_ascii_radar(
    const std::vector<std::pair<std::string, Vec2>>& positions,
    const std::map<std::string, CpaResult>& results,
    const Vec2& own_pos)
{
    if (positions.empty())
    {
        std::cout << "No targets for radar.\n";
        return;
    }

    const int gridSize = 41;
    const int center   = gridSize / 2;

    std::vector<std::vector<char>> grid(
        gridSize, std::vector<char>(gridSize, '.'));

    (void)own_pos;
    grid[center][center] = 'O';

    double maxAbs = 1.0;
    for (const auto& p : positions)
    {
        const auto& pos = p.second;
        maxAbs = std::max(maxAbs, std::fabs(pos.x));
        maxAbs = std::max(maxAbs, std::fabs(pos.y));
    }

    const double halfCells = static_cast<double>(center);
    const double scale     = (maxAbs <= 0.0) ? 1.0 : (maxAbs / halfCells);

    for (const auto& p : positions)
    {
        const std::string& id = p.first;
        const Vec2& pos = p.second;

        int col = center + static_cast<int>(std::round(pos.x / scale));
        int row = center - static_cast<int>(std::round(pos.y / scale));

        if (row < 0 || row >= gridSize || col < 0 || col >= gridSize) continue;

        auto it = results.find(id);
        bool risk = (it != results.end()) ? it->second.collision_risk : false;
        char symbol = risk ? 'C' : 'X';

        if (grid[row][col] != 'O')
            grid[row][col] = symbol;
    }

    std::cout << "=== Radar View (filtered final positions) ===\n";
    std::cout << "Top = +Y, Right = +X\n";
    std::cout << "Approx. scale: " << maxAbs
              << " m ~ " << halfCells << " cells\n\n";

    for (int r = 0; r < gridSize; ++r)
    {
        for (int c = 0; c < gridSize; ++c)
            std::cout << grid[r][c];
        std::cout << "\n";
    }
    std::cout << "\nLegend: O = ownship, X = target, C = collision risk\n\n";
}
