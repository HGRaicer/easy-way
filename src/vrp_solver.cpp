#include "vrp_solver.hpp"
#include "geocode.hpp"
#include "search.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdio>
#include <algorithm>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_parameters.h"

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
        int64_t& total_cost) {

        const int num_nodes = (int)dist_matrix.size();
        if (num_nodes < 2) return false;

        RoutingIndexManager manager(num_nodes, num_vehicles, RoutingIndexManager::NodeIndex{ 0 });
        RoutingModel routing(manager);

        const int distance_callback_index = routing.RegisterTransitCallback(
            [&dist_matrix, &manager](int64_t from_index, int64_t to_index) -> int64_t {
                int from_node = manager.IndexToNode(from_index).value();
                int to_node = manager.IndexToNode(to_index).value();
                return dist_matrix[from_node][to_node];
            }
        );

        const int time_callback_index = routing.RegisterTransitCallback(
            [&time_matrix, &manager](int64_t from_index, int64_t to_index) -> int64_t {
                int from_node = manager.IndexToNode(from_index).value();
                int to_node = manager.IndexToNode(to_index).value();
                return time_matrix[from_node][to_node];
            }
        );

        if (optimize_by_time) {
            routing.SetArcCostEvaluatorOfAllVehicles(time_callback_index);
        }
        else {
            routing.SetArcCostEvaluatorOfAllVehicles(distance_callback_index);
        }

        const int demand_callback_index = routing.RegisterUnaryTransitCallback(
            [&demands, &manager](int64_t from_index) -> int64_t {
                int node = manager.IndexToNode(from_index).value();
                return demands[node];
            }
        );
        routing.AddDimensionWithVehicleCapacity(demand_callback_index, 0, vehicle_capacity, true, "Capacity");

        int64_t time_horizon = 86400 * 1000;
        routing.AddDimension(time_callback_index, time_horizon, time_horizon, false, "Time");

        int64_t max_shift_duration = 12 * 3600 * 1000;
        for (int v = 0; v < num_vehicles; ++v) {
            routing.GetMutableDimension("Time")->SetSpanUpperBoundForVehicle(max_shift_duration, v);
        }
        routing.GetMutableDimension("Time")->SetSpanCostCoefficientForAllVehicles(100);

        for (int i = 1; i < num_nodes; ++i) {
            int64_t index = manager.NodeToIndex(RoutingIndexManager::NodeIndex{ i });
            int64_t tw_start = config.points[i].tw_start * 1000;
            int64_t tw_end = config.points[i].tw_end * 1000;
            routing.GetMutableDimension("Time")->CumulVar(index)->SetRange(tw_start, tw_end);
        }

        for (int v = 0; v < num_vehicles; ++v) {
            int64_t start_index = routing.Start(v);
            int64_t end_index = routing.End(v);
            routing.GetMutableDimension("Time")->CumulVar(start_index)->SetRange(0, time_horizon);
            routing.GetMutableDimension("Time")->CumulVar(end_index)->SetRange(0, time_horizon);
        }

        RoutingSearchParameters search_parameters = DefaultRoutingSearchParameters();
        search_parameters.set_first_solution_strategy(FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);
        search_parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
        search_parameters.mutable_time_limit()->set_seconds(5);

        const Assignment* solution = routing.SolveWithParameters(search_parameters);
        if (!solution) return false;

        total_cost = solution->ObjectiveValue();
        vehicle_routes.resize(num_vehicles);
        vehicle_arrival_times.resize(num_vehicles);
        const RoutingDimension* time_dimension = routing.GetMutableDimension("Time");

        for (int vehicle_id = 0; vehicle_id < num_vehicles; ++vehicle_id) {
            int64_t index = routing.Start(vehicle_id);
            while (!routing.IsEnd(index)) {
                int node_idx = manager.IndexToNode(index).value();
                vehicle_routes[vehicle_id].push_back(node_idx);
                vehicle_arrival_times[vehicle_id].push_back(solution->Value(time_dimension->CumulVar(index)));
                index = solution->Value(routing.NextVar(index));
            }
            int final_node_idx = manager.IndexToNode(index).value();
            vehicle_routes[vehicle_id].push_back(final_node_idx);
            vehicle_arrival_times[vehicle_id].push_back(solution->Value(time_dimension->CumulVar(index)));
        }
        return true;
    }
} // namespace operations_research

static bool parse_custom_csv(const std::string& path, NewVrpConfig& config, std::string& error) {
    std::ifstream file(path);
    if (!file.is_open()) { error = "Failed to open CSV file: " + path; return false; }
    std::string line;
    if (!std::getline(file, line)) { error = "CSV file is empty"; return false; }
    config.points.clear();
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string token;
        NewVrpPoint pt;
        try {
            if (!std::getline(ss, token, ',')) continue; pt.id = std::stoi(token);
            if (!std::getline(ss, token, ',')) continue; pt.lat = std::stod(token);
            if (!std::getline(ss, token, ',')) continue; pt.lon = std::stod(token);
            if (!std::getline(ss, token, ',')) continue; pt.demand = std::stoi(token);
            if (!std::getline(ss, token, ',')) continue; pt.tw_start = std::stoi(token);
            if (!std::getline(ss, token, ',')) continue; pt.tw_end = std::stoi(token);
            config.points.push_back(pt);
        }
        catch (...) { error = "Error parsing line: " + line; return false; }
    }
    return !config.points.empty();
}

static bool parse_fleet_csv(const std::string& path, std::vector<int64_t>& capacities, std::string& error) {
    std::ifstream file(path);
    if (!file.is_open()) { error = "Failed to open fleet CSV file: " + path; return false; }
    std::string line;
    if (!std::getline(file, line)) { error = "Fleet CSV file is empty"; return false; }
    capacities.clear();
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string token;
        try {
            if (!std::getline(ss, token, ',')) continue;
            if (!std::getline(ss, token, ',')) continue;
            capacities.push_back(std::stoll(token));
        }
        catch (...) { error = "Error parsing fleet line: " + line; return false; }
    }
    return !capacities.empty();
}

bool solve_new_vrp(const std::string& csvPath,
    const std::string& fleetCsvPath,
    const std::string& graphBinPath,
    const std::string& mapCsvPath,
    const std::string& outputPath,
    SearchMetric metric,
    bool full_output,
    std::string& error) {

    NewVrpConfig config;
    if (!parse_custom_csv(csvPath, config, error)) return false;

    std::vector<int64_t> capacities;
    if (!parse_fleet_csv(fleetCsvPath, capacities, error)) return false;

    int num_vehicles = static_cast<int>(capacities.size());
    config.num_vehicles = num_vehicles;

    int n = (int)config.points.size();
    if (n < 2) { error = "VRP needs at least 2 points (depot + customers)."; return false; }

    std::vector<std::pair<double, double>> vrp_coords;
    for (const auto& pt : config.points) { vrp_coords.push_back({ pt.lat, pt.lon }); }

    std::vector<uint32_t> node_ids;
    if (!operations_research::geocode_vector_of_coordinates(mapCsvPath, vrp_coords, node_ids, error)) return false;

    FILE* graph_file = fopen(graphBinPath.c_str(), "rb");
    if (!graph_file) { error = "Cannot open road graph binary: " + graphBinPath; return false; }

    std::vector<std::vector<int64_t>> time_matrix(n, std::vector<int64_t>(n, 0));
    std::vector<std::vector<int64_t>> dist_matrix(n, std::vector<int64_t>(n, 0));

    // =========================================================================
    // œ¿ ≈“Õ¿ﬂ √≈Õ≈–¿÷»ﬂ Ã¿“–»÷€ (Œœ“»Ã»«¿÷»ﬂ O(1) ¬Ã≈—“Œ O(N^2) ¬€«Œ¬Œ¬ ‘¿…ÀŒ¬)
    // =========================================================================
    std::string matIn = "temp_matrix_in.txt";
    std::string matOut = "temp_matrix_out.txt";

    {
        std::ofstream in(matIn);
        if (!in) { fclose(graph_file); error = "Cannot create temp matrix input file"; return false; }
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (i == j) continue;
                in << node_ids[i] << " " << node_ids[j] << "\n";
            }
        }
    }

    FILE* inf = fopen(matIn.c_str(), "r");
    FILE* outf = fopen(matOut.c_str(), "w");
    if (!inf || !outf) {
        if (inf) fclose(inf); if (outf) fclose(outf);
        fclose(graph_file); error = "Failed to open temporary batch files for matrix."; return false;
    }

    fseek(graph_file, 0, SEEK_SET);
    run_search(graph_file, inf, outf, false, metric); // full_output = false ‰Îˇ Ï‡ÚËˆ˚
    fclose(inf); fclose(outf);

    std::ifstream out(matOut);
    if (!out.is_open()) { fclose(graph_file); error = "Failed to open temporary matrix output."; return false; }

    std::string raw_line;
    std::getline(out, raw_line); // œÓÔÛÒÍ ·‡ÌÌÂ‡ "=== A* Optimization Result ==="

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == j) {
                time_matrix[i][j] = 0;
                dist_matrix[i][j] = 0;
                continue;
            }
            if (std::getline(out, raw_line)) {
                std::stringstream ss(raw_line);
                double t_raw = 0.0, d_raw = 0.0;
                if (ss >> t_raw >> d_raw) {
                    time_matrix[i][j] = static_cast<int64_t>(t_raw * 1000.0);
                    dist_matrix[i][j] = static_cast<int64_t>(d_raw * 1000.0);
                }
                else {
                    time_matrix[i][j] = 3600 * 1000;
                    dist_matrix[i][j] = 50000 * 1000;
                }
            }
            else {
                time_matrix[i][j] = 3600 * 1000;
                dist_matrix[i][j] = 50000 * 1000;
            }
        }
    }
    out.close();
    std::remove(matIn.c_str());
    std::remove(matOut.c_str());

    // =========================================================================

    std::vector<int> demands(n);
    for (int i = 0; i < n; ++i) { demands[i] = config.points[i].demand; }

    std::vector<std::vector<int>> vehicle_routes;
    std::vector<std::vector<int64_t>> vehicle_arrival_times;
    int64_t total_optimization_cost = 0;

    bool optimize_by_time = (metric == SearchMetric::Time);

    if (!operations_research::InternalSolveVRP(dist_matrix, time_matrix, demands, config, num_vehicles,
        capacities, optimize_by_time, vehicle_routes, vehicle_arrival_times, total_optimization_cost)) {
        fclose(graph_file);
        error = "Google OR-Tools failed to find a valid vehicle routing solution.";
        return false;
    }

    std::ofstream out_res(outputPath);
    if (!out_res.is_open()) { fclose(graph_file); error = "Cannot open output file: " + outputPath; return false; }

    int64_t global_pure_transit_time = 0;
    int64_t global_dist = 0;
    int64_t max_operation_endtime = 0;

    std::vector<int64_t> fleet_pure_times(num_vehicles, 0);
    std::vector<int64_t> fleet_dists(num_vehicles, 0);

    for (int v = 0; v < num_vehicles; ++v) {
        const auto& route = vehicle_routes[v];
        if (route.size() < 2) continue;
        for (size_t i = 0; i < route.size() - 1; ++i) {
            int from_node = route[i];
            int to_node = route[i + 1];
            fleet_pure_times[v] += time_matrix[from_node][to_node];
            fleet_dists[v] += dist_matrix[from_node][to_node];
        }
        global_pure_transit_time += fleet_pure_times[v];
        global_dist += fleet_dists[v];

        if (!vehicle_arrival_times[v].empty()) {
            int64_t end_time = vehicle_arrival_times[v].back();
            if (end_time > max_operation_endtime) { max_operation_endtime = end_time; }
        }
    }

    out_res << "=== New VRP Routing Optimization Result ===\n";
    out_res << "Optimized Target: " << (optimize_by_time ? "TIME" : "DISTANCE") << "\n";
    out_res << "Global Pure Drive Time (seconds): " << (global_pure_transit_time / 1000.0) << "\n";
    out_res << "Global Fleet Distance (meters): " << (global_dist / 1000.0) << "\n";
    out_res << "Real Complete Operations Time (seconds): " << max_operation_endtime / 1000.0 << " (Makespan)\n";
    out_res << "Vehicles Allocated: " << num_vehicles << "\n\n";

    for (size_t v = 0; v < vehicle_routes.size(); ++v) {
        int64_t total_route_time = 0;
        int64_t start_time = 0;
        int64_t end_time = 0;

        if (!vehicle_arrival_times[v].empty()) {
            start_time = vehicle_arrival_times[v].front();
            end_time = vehicle_arrival_times[v].back();
            total_route_time = end_time - start_time;
        }
        int64_t waiting_time = total_route_time - (fleet_pure_times[v]);
        if (waiting_time < 0) waiting_time = 0;

        out_res << "Vehicle " << (v + 1) << " Route Summary:\n"
            << "  -> Start Time (Departure from Depot): " << start_time / 1000.0 << " seconds\n"
            << "  -> End Time (Return to Depot): " << end_time / 1000.0 << " seconds\n"
            << "  -> Total Duration (incl. waiting): " << total_route_time / 1000.0 << " seconds\n"
            << "  -> Pure Driving Time: " << (fleet_pure_times[v] / 1000.0) << " seconds\n"
            << "  -> Waiting at windows: " << waiting_time / 1000.0 << " seconds\n"
            << "  -> Distance: " << (fleet_dists[v] / 1000.0) << " meters\n"
            << "  -> Max Capacity Limit: " << capacities[v] << "\n";

        out_res << "  Nodes sequence:\n";
        for (size_t i = 0; i < vehicle_routes[v].size(); ++i) {
            int node = vehicle_routes[v][i];
            int64_t arr_sec = vehicle_arrival_times[v][i];
            const auto& pt = config.points[node];
            uint32_t dense_id = node_ids[node];

            out_res << "    * Arrive: " << arr_sec / 1000.0 << "s | Point ID: " << pt.id
                << " [Dense ID: " << dense_id << "] (Lat: " << pt.lat << ", Lon: " << pt.lon << ")";

            if (node == 0) {
                if (i == 0) out_res << " [START DEPOT]";
                else out_res << " [END DEPOT]";
            }
            else {
                out_res << " [Demand: " << pt.demand << ", Window: " << pt.tw_start << "-" << pt.tw_end << "]";
            }
            out_res << "\n";
        }
        out_res << "\n";
    }

    // =========================================================================
    // œ¿ ≈“ÕŒ≈ œŒ—“–Œ≈Õ»≈ œŒÀÕ€’ “–≈ Œ¬ √≈ŒÃ≈“–»» (”¡»–¿≈“ ƒ≈—ﬂ“ » ¬€«Œ¬Œ¬ ‘¿…ÀŒ¬)
    // =========================================================================
    if (full_output) {
        out_res << "=== Full Fleet Dense ID Tracks ===\n\n";

        std::string vIn = "temp_v_track_in.txt";
        std::string vOut = "temp_v_track_out.txt";

        {
            std::ofstream in(vIn);
            if (!in) { fclose(graph_file); error = "Cannot create temp track input file"; return false; }
            for (size_t v = 0; v < vehicle_routes.size(); ++v) {
                if (vehicle_routes[v].size() < 2) continue;
                for (size_t i = 0; i < vehicle_routes[v].size() - 1; ++i) {
                    in << node_ids[vehicle_routes[v][i]] << " " << node_ids[vehicle_routes[v][i + 1]] << "\n";
                }
            }
        }

        FILE* inf_track = fopen(vIn.c_str(), "r");
        FILE* outf_track = fopen(vOut.c_str(), "w");
        if (inf_track && outf_track) {
            fseek(graph_file, 0, SEEK_SET);
            run_search(graph_file, inf_track, outf_track, true, metric); // full_output = true ‰Îˇ „ÂÓÏÂÚËË
            fclose(inf_track); fclose(outf_track);
        }
        else {
            if (inf_track) fclose(inf_track); if (outf_track) fclose(outf_track);
        }

        std::ifstream path_stream(vOut);
        if (path_stream.is_open()) {
            std::string raw_route_line;
            std::getline(path_stream, raw_route_line); // œÓÔÛÒÍ ·‡ÌÌÂ‡

            std::vector<uint32_t> all_nodes;
            all_nodes.reserve(10000);

            for (size_t v = 0; v < vehicle_routes.size(); ++v) {
                if (vehicle_routes[v].size() < 2) continue;

                double total_time = 0.0;
                double total_distance = 0.0;
                all_nodes.clear();

                for (size_t i = 0; i < vehicle_routes[v].size() - 1; ++i) {
                    if (std::getline(path_stream, raw_route_line)) {
                        std::stringstream ss(raw_route_line);
                        double seg_time, seg_dist;
                        uint32_t seg_cnt;

                        if (ss >> seg_time >> seg_dist >> seg_cnt) {
                            total_time += seg_time;
                            total_distance += seg_dist;

                            uint32_t osm_node_id;
                            while (ss >> osm_node_id) {
                                if (all_nodes.empty() || osm_node_id != all_nodes.back()) {
                                    all_nodes.push_back(osm_node_id);
                                }
                            }
                        }
                    }
                }

                // ¬˚‚Ó‰ ÂÁÛÎ¸Ú‡ÚÓ‚ ‰Îˇ Ï‡¯ËÌ˚
                out_res << "Vehicle " << (v + 1) << " Route Sequence (Full Dense ID Graph Path):\n  ";
                out_res << total_time << " " << total_distance << " " << all_nodes.size() << " ";
                for (uint32_t node_id : all_nodes) {
                    out_res << node_id << " ";
                }
                out_res << "\n\n";
            }
            path_stream.close();
        }
        std::remove(vIn.c_str());
        std::remove(vOut.c_str());
    }

    fclose(graph_file);
    return true;
}