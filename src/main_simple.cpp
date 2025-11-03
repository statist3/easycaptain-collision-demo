#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <sstream>
#include <iomanip>


struct Target 
{
    int id;
    double x;
    double y;
    double speed;
    double cource_deg;
};

struct CpaResult
{
    double cpa_distance;
    double tcpa;
    bool collision_risk;
    bool closing;
    bool valid;
};

constexpr double CPA_THRESHOLD_METERS = 50.0;
constexpr double TCPA_THRESHOLD_SECONDS = 30.0;


bool load_targets_from_csv(const std::string& path, std::vector<Target>& targets)
{
    std::ifstream file(path);
    if (!file)
    {
        std::cerr << "Failed to open file: " << path << "\n";
        return false;
    }

    std::string line;

    if(!std::getline(file, line))
    {
        std::cerr << "Empty file or read error: " << path << "\n";
        return false;
    }

    while (std::getline(file, line))
    {
        if(line.empty())
            continue;

        std::stringstream ss(line);
        std::string field;
        std::vector<std::string> fields;

        while (std::getline(ss, field, ','))
        {
            fields.push_back(field);
        }

        if (fields.size() != 5)
        {
            std::cerr << "Invalid CSV line (expected 5 columns): " << line << "\n";
            continue;
        }

        Target t;
        t.id = std::stoi(fields[0]);
        t.x = std::stod(fields[1]);
        t.y = std::stod(fields[2]);
        t.speed = std::stod(fields[3]);
        t.cource_deg = std::stod(fields[4]);

        targets.push_back(t);
    }

    return true;
}

std::pair<double, double> course_to_velocity(double speed, double course_deg)
{
    double rad = course_deg * M_PI / 180.0;
    double vx = speed * std::cos(rad);
    double vy = speed * std::sin(rad);
    return std::make_pair(vx, vy);
}


CpaResult compute_cpa(
    const std::pair<double, double>& own_pos,
    const std::pair<double, double>& own_vel,
    const std::pair<double, double>& tgt_pos,
    const std::pair<double, double>& tgt_vel)
{
    double x_own = own_pos.first;
    double y_own = own_pos.second;

    double vx_own = own_vel.first;
    double vy_own = own_vel.second;

    double x_t = tgt_pos.first;
    double y_t = tgt_pos.second;

    double vx_t = tgt_vel.first;
    double vy_t = tgt_vel.second;

    // relative position
    double rx = x_t - x_own;
    double ry = y_t - y_own;

    // relative velocity
    double vx_rel = vx_t - vx_own;
    double vy_rel = vy_t - vy_own;

    double v2 = vx_rel * vx_rel + vy_rel * vy_rel;
    const double eps = 1e-9;

    if(v2 < eps)
    {
        double dist = std::hypot(rx, ry);
        return CpaResult{
            dist,
            0.0,
            false,
            false,
            false
        };
    }

    double dot = rx * vx_rel + ry * vy_rel;
    double tcpa = -dot / v2;

    // position at the time of CPA
    double rx_cpa = rx + vx_rel * tcpa;
    double ry_cpa = ry + vy_rel * tcpa;

    double cpa_dist = std::hypot(rx_cpa, ry_cpa);

    bool closing = (tcpa >= 0.0);
    bool collision_risk =
        closing &&
        (cpa_dist < CPA_THRESHOLD_METERS) &&
        (tcpa < TCPA_THRESHOLD_SECONDS);

    return CpaResult{
        cpa_dist,
        tcpa,
        collision_risk,
        closing,
        true
    };
}

void print_ascii_radar(const std::vector<Target>& targets,
                       const std::vector<CpaResult>& results)
{
    if(targets.empty())
    {
        std::cout << "No targets for radar.\n";
        return;
    }

    const int gridSize = 41;
    const int center = gridSize / 2;

    std::vector<std::vector<char>> grid(
        gridSize,
        std::vector<char>(gridSize, '.'));
    
    grid[center][center] = '0';

    double maxAbsCoord = 1.0;
    for(const auto& t : targets)
    {
        maxAbsCoord = std::max(maxAbsCoord, std::fabs(t.x));
        maxAbsCoord = std::max(maxAbsCoord, std::fabs(t.y));
    }

    const double halfCells = static_cast<double>(center);
    const double scale = (maxAbsCoord <= 0.0) ? 1.0 : (maxAbsCoord / halfCells);

    for(std::size_t i =0; i < targets.size(); ++i)
    {
        const auto& t = targets[i];
        const auto& r = results[i];

        double x = t.x;
        double y = t.y;

        int col = center + static_cast<int>(std::round(x / scale));
        int row = center - static_cast<int>(std::round(y/ scale));

        if(row < 0 || row >= gridSize || col < 0 || col >= gridSize)
        {
            continue; // out of radar range
        }
        char symbol = r.collision_risk ? 'C' : 'X';
        if(grid[row][col] != '0')
        {
            grid[row][col] = symbol;
        }

        std::cout << "===Radar View===\n";
        std::cout << "Top = +Y, Right = +X\n";
        std::cout << "Approx. scale: " << maxAbsCoord
                  << " m ~ " << halfCells << " cells\n\n";

        for(int row = 0; row < gridSize; ++row)
        {
            for(int col = 0; col < gridSize; ++col)
            {
                std::cout << grid[row][col];
            }
            std::cout << "\n";
        }
        std::cout << "\nLegend: O = ownship, X = target, C = collision risk\n\n";
    }
}


void print_results(const std::vector<Target>& targets, 
                   const std::vector<CpaResult>& results)
{
    std::cout << std::fixed << std::setprecision(1);

    for(std::size_t i = 0; i < targets.size(); ++i)
    {
        const auto& t = targets[i];
        const auto& r = results[i];

        std::string status = r.collision_risk ? "COLLISION RISK" : "Safe";


        std::cout 
            << "Target " << t.id << ": "
            << "CPA = " << r.cpa_distance << "m, "
            << "TCPA = " << r.tcpa << "s "
            << status 
            << "\n";
    }
}


int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        std::cerr << "Usage: cpa_risk <path/to/targets.csv>\n";
        return 1;
    }

    std::string csv_path = argv[1];

    std::vector<Target> targets;
    if(!load_targets_from_csv(csv_path, targets))
    {
        return 1;
    }

    if(targets.empty())
    {
        std::cerr << "No targets loaded from CSV.\n";
        return 1;
    }

    std::pair<double, double> own_pos{0.0, 0.0};
    double own_speed = 20.0;
    double own_course_deg = 30.0;

    auto own_vel = course_to_velocity(own_speed, own_course_deg);

    std::vector<CpaResult> results;
    results.reserve(targets.size());

    for(const auto& t : targets)
    {
        std::pair<double, double> tgt_pos{t.x, t.y};
        auto tgt_vel = course_to_velocity(t.speed, t.cource_deg);
        results.push_back(compute_cpa(own_pos, own_vel, tgt_pos, tgt_vel));
    }

    print_results(targets, results);
    print_ascii_radar(targets, results);

    return 0;
}