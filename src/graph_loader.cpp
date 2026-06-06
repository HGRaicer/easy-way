#include "graph_loader.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <fstream>
#include <sstream>

struct GraphHeaderV2 {
    uint32_t version{};
    uint64_t N{};
    uint32_t flags{};
};

static bool read_header(FILE* f, GraphHeaderV2& h) {
    unsigned char magic[4];
    if (fread(magic, 1, 4, f) != 4 || std::memcmp(magic, "GBIN", 4) != 0) return false;
    if (fread(&h.version, sizeof(uint32_t), 1, f) != 1) return false;
    if (h.version != 2) return false;
    if (fread(&h.N, sizeof(uint64_t), 1, f) != 1) return false;
    if (fread(&h.flags, sizeof(uint32_t), 1, f) != 1) return false;
    return true;
}

LoadedGraph load_graph_bin(const std::string& bin_path) {
    LoadedGraph g;
    FILE* f = fopen(bin_path.c_str(), "rb");
    if (!f) return g;

    GraphHeaderV2 h{};
    if (!read_header(f, h)) {
        fclose(f);
        return g;
    }

    g.N = static_cast<uint32_t>(h.N);
    g.speed_present = (h.flags & 1u) != 0;

    // skip ids
    fseek(f, static_cast<long long>(g.N) * sizeof(uint32_t), SEEK_CUR);

    // coords
    g.coords.resize(g.N);
    for (uint32_t i = 0; i < g.N; ++i) {
        fread(&g.coords[i].lat, sizeof(double), 1, f);
        fread(&g.coords[i].lon, sizeof(double), 1, f);
    }

    // offsets
    g.offsets.resize(g.N);
    fread(g.offsets.data(), sizeof(uint32_t), g.N, f);

    const uint32_t M = g.offsets.empty() ? 0 : g.offsets.back();

    // adjacency
    g.adjacency.resize(M);
    fread(g.adjacency.data(), sizeof(uint32_t), M, f);

    // speeds
    if (g.speed_present && M > 0) {
        g.speeds.resize(M);
        fread(g.speeds.data(), sizeof(float), M, f);
    }

    fclose(f);
    return g;
}

uint32_t LoadedGraph::find_nearest(double lat, double lon) const {
    if (N == 0) return 0;

    uint32_t best = 0;
    double best_d = std::numeric_limits<double>::max();

    for (uint32_t i = 0; i < N; ++i) {
        const double dx = coords[i].lon - lon;
        const double dy = coords[i].lat - lat;
        const double d = dx * dx + dy * dy;
        if (d < best_d) {
            best_d = d;
            best = i;
        }
    }
    return best;
}

std::string trim(const std::string& s) {
    auto start = s.find_first_not_of(" \t\r\n");
    auto end = s.find_last_not_of(" \t\r\n");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
}

void load_routes_from_file(const std::string& result_txt, std::vector<Route>& routes_out) {
    static const std::array<Rgba, 10> kRoutePalette = { {
        { 74, 144, 226, 255}, { 80, 190, 170, 255}, { 156, 120, 210, 255},
        { 230, 145,  72, 255}, { 105, 185, 110, 255}, { 214, 105, 135, 255},
        {  88, 170, 205, 255}, { 218, 190,  85, 255}, { 170, 135, 100, 255},
        { 130, 160, 230, 255}
    } };

    std::ifstream in(result_txt);
    if (!in) return;

    std::vector<std::vector<RouteTarget>> vrp_targets_by_vehicle;
    std::vector<RouteTarget> tsp_targets;
    int current_summary_vehicle_idx = -1;

    std::string line;
    while (std::getline(in, line)) {
        std::string t_line = trim(line);
        if (t_line.empty()) continue;

        // --- БЛОК ПАРСИНГА ТОЧЕК ПОСЕЩЕНИЯ (TARGETS) ---

        // Для VRP: собираем точки из секции "Vehicle X Route Summary"
        if (t_line.find("Vehicle ") == 0 && t_line.find(" Route Summary:") != std::string::npos) {
            if (sscanf(t_line.c_str(), "Vehicle %d", &current_summary_vehicle_idx) == 1) {
                current_summary_vehicle_idx--; // в 0-based
                if (current_summary_vehicle_idx >= (int)vrp_targets_by_vehicle.size())
                    vrp_targets_by_vehicle.resize(current_summary_vehicle_idx + 1);
            }
            continue;
        }

        // Для VRP: парсим строку с Dense ID внутри Summary
        if (t_line.find("[Dense ID: ") != std::string::npos && t_line.find("* Arrive:") != std::string::npos) {
            uint32_t dense_id = 0;
            double arr_time = 0;
            size_t p_dens = t_line.find("[Dense ID: ");
            size_t p_arr = t_line.find("* Arrive: ");

            sscanf(t_line.c_str() + p_dens, "[Dense ID: %u]", &dense_id);
            sscanf(t_line.c_str() + p_arr, "* Arrive: %lfs", &arr_time);

            if (current_summary_vehicle_idx >= 0) {
                RouteTarget tgt;
                tgt.dense_id = (dense_id > 0) ? (dense_id - 1) : 0;
                tgt.arrival_time_s = arr_time;
                tgt.is_depot = (t_line.find("DEPOT") != std::string::npos);
                vrp_targets_by_vehicle[current_summary_vehicle_idx].push_back(tgt);
            }
            continue;
        }

        // Для TSP: собираем точки из секции "Optimal order"
        if (t_line.find("-> node_id ") != std::string::npos) {
            uint32_t node_id = 0;
            size_t pos = t_line.find("node_id ");
            if (sscanf(t_line.c_str() + pos, "node_id %u", &node_id) == 1) {
                RouteTarget tgt;
                tgt.dense_id = (node_id > 0) ? (node_id - 1) : 0;
                tgt.arrival_time_s = 0.0; // Для TSP время не критично в легенде
                tgt.is_depot = (t_line.find("(BASE)") != std::string::npos);
                tsp_targets.push_back(tgt);
            }
            continue;
        }

        // --- БЛОК ПАРСИНГА ГЕОМЕТРИИ (NODES) ---

        Route r;
        bool has_route = false;
        int active_vehicle_id = -1;

        // 1. Формат TSP (сегменты пути)
        if (t_line.find("Route from Point") == 0) {
            double tm = 0, ds = 0;
            if (sscanf(t_line.c_str(), "Route from Point %*d to Point %*d (Time: %lfs, Dist: %lfm)", &tm, &ds) >= 2) {
                r.total_time_s = tm;
                r.total_dist_m = ds;
                if (std::getline(in, line)) {
                    std::stringstream ss(line);
                    uint32_t id;
                    while (ss >> id) r.nodes.push_back(id > 0 ? id - 1 : 0);
                    has_route = true;
                }
            }
        }
        // 2. Формат VRP (цельный трек машины)
        else if (t_line.find("Vehicle") == 0 && t_line.find("Route Sequence") != std::string::npos) {
            sscanf(t_line.c_str(), "Vehicle %d", &active_vehicle_id);
            active_vehicle_id--;
            if (std::getline(in, line)) {
                std::stringstream ss(line);
                uint32_t count;
                if (ss >> r.total_time_s >> r.total_dist_m >> count) {
                    uint32_t id;
                    while (ss >> id) r.nodes.push_back(id > 0 ? id - 1 : 0);
                    has_route = true;
                }
            }
        }
        // 3. Формат A* (простая числовая строка)
        else if (std::isdigit((unsigned char)t_line[0])) {
            std::stringstream ss(t_line);
            uint32_t count;
            if (ss >> r.total_time_s >> r.total_dist_m >> count) {
                uint32_t id;
                while (ss >> id) r.nodes.push_back(id > 0 ? id - 1 : 0);
                has_route = true;
            }
        }

        if (has_route && !r.nodes.empty()) {
            size_t idx = routes_out.size();
            r.label = "Route " + std::to_string(idx + 1);
            r.color = kRoutePalette[idx % kRoutePalette.size()];

            // Привязка таргетов (флажков)
            if (active_vehicle_id >= 0 && active_vehicle_id < (int)vrp_targets_by_vehicle.size()) {
                // Это VRP трек - берем таргеты именно этой машины
                r.targets = vrp_targets_by_vehicle[active_vehicle_id];
            }
            else if (!tsp_targets.empty()) {
                // Это TSP трек - привязываем все собранные точки TSP
                // (Для TSP обычно один общий набор точек на все сегменты)
                r.targets = tsp_targets;
            }

            routes_out.push_back(std::move(r));
        }
    }
}