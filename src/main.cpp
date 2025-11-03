#include <iostream>
#include <iomanip>
#include <map>
#include <vector>
#include <string>
#include <cstdlib>

#include "kalman.h"
#include "cpa.h"
#include "io.h"
#include "radar.h"
#include "json_writer.h"

static void print_usage()
{
    std::cerr << "Usage: cpa_risk <csv_path> [--own-speed V] [--own-course DEG] [--json-out file]\n";
    std::cerr << "\nCSV format (time series):\n"
              << "time,id,x,y,speed,course\n"
              << "0,1,100,50,5,180\n"
              << "1,1,95,50,5,180\n"
              << "2,1,90,50,5,180\n";
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        print_usage();
        return 1;
    }

    std::string csv_path;
    std::string json_path;
    double own_speed      = 20.0;  // defaults
    double own_course_deg = 30.0;  // defaults

    // simple arg parser
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg.rfind("--", 0) == 0)
        {
            if (arg == "--own-speed")
            {
                if (i + 1 >= argc)
                {
                    std::cerr << "--own-speed requires a value\n";
                    return 1;
                }
                own_speed = std::strtod(argv[++i], nullptr);
            }
            else if (arg == "--own-course")
            {
                if (i + 1 >= argc)
                {
                    std::cerr << "--own-course requires a value\n";
                    return 1;
                }
                own_course_deg = std::strtod(argv[++i], nullptr);
            }
            else if (arg == "--json-out")
            {
                if (i + 1 >= argc)
                {
                    std::cerr << "--json-out requires a file path\n";
                    return 1;
                }
                json_path = argv[++i];
            }
            else
            {
                std::cerr << "Unknown option: " << arg << "\n";
                print_usage();
                return 1;
            }
        }
        else
        {
            if (csv_path.empty())
                csv_path = arg;
            else
            {
                std::cerr << "Unexpected extra argument: " << arg << "\n";
                print_usage();
                return 1;
            }
        }
    }

    if (csv_path.empty())
    {
        std::cerr << "CSV path is required.\n";
        print_usage();
        return 1;
    }

    // time-series: id -> vector<Measurement>
    std::map<std::string, std::vector<Measurement>> series;
    if (!load_timeseries_from_csv(csv_path, series))
        return 1;

    if (series.empty())
    {
        std::cerr << "No data loaded.\n";
        return 1;
    }

    Vec2 own_pos{0.0, 0.0};
    Vec2 own_vel = course_to_velocity(own_speed, own_course_deg);

    // run filter for each ID
    std::map<std::string, CpaResult> final_results;
    std::map<std::string, Vec2> final_positions;
    std::map<std::string, Vec2> final_velocities;
    std::vector<std::pair<std::string, Vec2>> radar_positions;

    for (const auto& kv : series)
    {
        const std::string& id = kv.first;
        const auto& seq = kv.second;
        if (seq.empty()) continue;

        KalmanFilter2D kf;
        Vec2 v0 = course_to_velocity(seq.front().speed, seq.front().course_deg);
        kf.init(seq.front().x, seq.front().y, v0.x, v0.y);

        double prev_time = seq.front().time;

        for (std::size_t i = 0; i < seq.size(); ++i)
        {
            double t   = seq[i].time;
            double dt  = t - prev_time;
            if (dt < 0) dt = 0.0;

            kf.predict(dt);
            kf.update(seq[i].x, seq[i].y);

            prev_time = t;
        }

        Vec2 filt_pos{kf.getX(),  kf.getY()};
        Vec2 filt_vel{kf.getVx(), kf.getVy()};

        CpaResult res = compute_cpa(own_pos, own_vel, filt_pos, filt_vel);

        final_results[id]   = res;
        final_positions[id] = filt_pos;
        final_velocities[id]= filt_vel;

        radar_positions.push_back({id, filt_pos});
    }

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "=== CPA / TCPA Results (Kalman, final state per id) ===\n";
    std::cout << std::left
              << std::setw(8)  << "ID"
              << std::setw(12) << "CPA [m]"
              << std::setw(12) << "TCPA [s]"
              << "Status\n";
    std::cout << std::string(8+12+12+12, '-') << "\n";

    for (const auto& kv : series)
    {
        const std::string& id = kv.first;
        const auto& r = final_results[id];

        std::string status;
        if (!r.valid)              status = "No relative motion";
        else if (!r.closing)       status = "Diverging";
        else if (r.collision_risk) status = "COLLISION RISK";
        else                       status = "Safe";

        std::cout << std::left
                  << std::setw(8)  << id
                  << std::setw(12) << r.cpa_distance
                  << std::setw(12) << r.tcpa
                  << status << "\n";
    }
    std::cout << "\n";

    print_ascii_radar(radar_positions, final_results, own_pos);

    if (!json_path.empty())
    {
        write_json(json_path,
                   series,
                   final_results,
                   final_positions,
                   final_velocities,
                   own_pos,
                   own_vel);
    }

    return 0;
}
