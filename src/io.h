#pragma once
#include <string>
#include <vector>
#include <map>

struct Measurement
{
    double time;        // seconds
    std::string id;
    double x;
    double y;
    double speed;
    double course_deg;
};

// CSV format:
// time,id,x,y,speed,course
// return: id -> vector measurements by time
bool load_timeseries_from_csv(const std::string& path,
                              std::map<std::string, std::vector<Measurement>>& out);

std::string trim_copy(const std::string& s);
