#include "json_writer.h"

#include <fstream>
#include <iostream>
#include <iomanip>

void write_json(
    const std::string& path,
    const std::map<std::string, std::vector<Measurement>>& series,
    const std::map<std::string, CpaResult>& final_results,
    const std::map<std::string, Vec2>& final_positions,
    const std::map<std::string, Vec2>& final_velocities,
    const Vec2& own_pos,
    const Vec2& own_vel)
{
    std::ofstream out(path);
    if (!out)
    {
        std::cerr << "Failed to open JSON file for writing: " << path << "\n";
        return;
    }

    out << std::fixed << std::setprecision(3);
    out << "{\n";

    // ownship block
    out << "  \"ownship\": {\n";
    out << "    \"position\": { \"x\": " << own_pos.x
        << ", \"y\": " << own_pos.y << " },\n";
    out << "    \"velocity\": { \"vx\": " << own_vel.x
        << ", \"vy\": " << own_vel.y << " }\n";
    out << "  },\n";

    // targets
    out << "  \"targets\": [\n";

    bool first_target = true;
    for (const auto& kv : series)
    {
        const std::string& id = kv.first;
        const auto& seq = kv.second;

        if (!first_target)
            out << ",\n";
        first_target = false;

        const auto& cpa = final_results.at(id);
        const auto& pos = final_positions.at(id);
        const auto& vel = final_velocities.at(id);

        out << "    {\n";
        out << "      \"id\": \"" << id << "\",\n";

        // measurements array
        out << "      \"measurements\": [\n";
        for (std::size_t i = 0; i < seq.size(); ++i)
        {
            const auto& m = seq[i];
            out << "        { "
                << "\"time\": " << m.time
                << ", \"x\": " << m.x
                << ", \"y\": " << m.y
                << ", \"speed\": " << m.speed
                << ", \"course\": " << m.course_deg
                << " }";
            if (i + 1 != seq.size()) out << ",";
            out << "\n";
        }
        out << "      ],\n";

        // filtered state
        out << "      \"filtered_state\": {\n";
        out << "        \"x\": " << pos.x << ",\n";
        out << "        \"y\": " << pos.y << ",\n";
        out << "        \"vx\": " << vel.x << ",\n";
        out << "        \"vy\": " << vel.y << "\n";
        out << "      },\n";

        // CPA info
        out << "      \"cpa\": {\n";
        out << "        \"distance\": " << cpa.cpa_distance << ",\n";
        out << "        \"tcpa\": " << cpa.tcpa << ",\n";
        out << "        \"collision_risk\": "
            << (cpa.collision_risk ? "true" : "false") << ",\n";
        out << "        \"closing\": "
            << (cpa.closing ? "true" : "false") << ",\n";
        out << "        \"valid\": "
            << (cpa.valid ? "true" : "false") << "\n";
        out << "      }\n";

        out << "    ";
    }

    out << "\n  ]\n";
    out << "}\n";

    std::cout << "JSON saved to " << path << "\n";
}
