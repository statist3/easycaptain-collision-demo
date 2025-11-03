#include "io.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>

static inline void trim_inplace(std::string& s)
{
    auto isspace2 = [](unsigned char c){ return std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [&](unsigned char c){ return !isspace2(c); }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [&](unsigned char c){ return !isspace2(c); }).base(), s.end());
}

std::string trim_copy(const std::string& s)
{
    std::string t = s;
    trim_inplace(t);
    return t;
}

bool load_timeseries_from_csv(const std::string& path,
                              std::map<std::string, std::vector<Measurement>>& out)
{
    std::ifstream file(path);
    if (!file)
    {
        std::cerr << "Failed to open file: " << path << "\n";
        return false;
    }

    std::string line;
    if (!std::getline(file, line))
    {
        std::cerr << "Empty file or read error: " << path << "\n";
        return false;
    }

    // we are waiting for the title: time,id,x,y,speed,course

    while (std::getline(file, line))
    {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string field;
        std::vector<std::string> fields;
        while (std::getline(ss, field, ','))
        {
            trim_inplace(field);
            fields.push_back(field);
        }

        if (fields.size() != 6)
        {
            std::cerr << "Invalid CSV line (expected 6 columns): " << line << "\n";
            continue;
        }

        Measurement m;
        try
        {
            m.time       = std::stod(fields[0]);
            m.id         = fields[1];
            m.x          = std::stod(fields[2]);
            m.y          = std::stod(fields[3]);
            m.speed      = std::stod(fields[4]);
            m.course_deg = std::stod(fields[5]);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Parse error: " << e.what() << " in line: " << line << "\n";
            continue;
        }

        out[m.id].push_back(m);
    }

    // sort by time within each id
    for (auto& kv : out)
    {
        auto& vec = kv.second;
        std::sort(vec.begin(), vec.end(),
                  [](const Measurement& a, const Measurement& b){ return a.time < b.time; });
    }

    return true;
}
