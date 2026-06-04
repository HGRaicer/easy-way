#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include "search.h"

struct NewVrpPoint {
    int id;
    double lat;
    double lon;
    int demand;
    int tw_start;
    int tw_end;
};

struct NewVrpConfig {
    std::vector<NewVrpPoint> points;
    int num_vehicles;
};

namespace operations_research {
    bool InternalSolveVRP(const std::vector<std::vector<int64_t>>& dist_matrix,
        const std::vector<std::vector<int64_t>>& time_matrix,
        const std::vector<int>& demands,
        const NewVrpConfig& config,
        int num_vehicles,
        std::vector<int64_t> vehicle_capacity,
        bool optimize_by_time,
        std::vector<std::vector<int>>& vehicle_routes,
        std::vector<std::vector<int64_t>>& vehicle_arrival_times,
        int64_t& total_cost);
}

bool solve_new_vrp(const std::string& csvPath,
    const std::string& fleetCsvPath,
    const std::string& graphBinPath,
    const std::string& mapCsvPath,
    const std::string& outputPath,
    SearchMetric metric,
    bool full_output,
    std::string& error);