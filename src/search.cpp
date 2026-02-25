#include "search.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <cstdio>

constexpr double PI = 3.141592653589793;
constexpr double EARTH_RADIUS = 6371000.0;

static double haversine(const SearchNode& a, const SearchNode& b) noexcept {
    double dlat = (b.lat - a.lat) * PI / 180.0;
    double dlon = (b.lon - a.lon) * PI / 180.0;

    double lat1_rad = a.lat * PI / 180.0;
    double lat2_rad = b.lat * PI / 180.0;

    double sin_dlat = sin(dlat * 0.5);
    double sin_dlon = sin(dlon * 0.5);

    double a_val = sin_dlat * sin_dlat + cos(lat1_rad) * cos(lat2_rad) * sin_dlon * sin_dlon;
    return 2.0 * EARTH_RADIUS * asin(sqrt(a_val));
}

static size_t binarySearch(const std::vector<uint32_t>& ids, uint32_t target) noexcept {
    size_t left = 0, right = ids.size();
    while (left < right) {
        size_t mid = left + (right - left) / 2;
        if (ids[mid] < target) left = mid + 1;
        else right = mid;
    }
    return (left < ids.size() && ids[left] == target) ? left : size_t(-1);
}

static SearchNode getNode(FILE* file, uint32_t coord_offset, size_t index) noexcept {
    fseek(file, coord_offset + static_cast<long>(index * 2 * sizeof(double)), SEEK_SET);
    SearchNode node{};
    fread(&node.lat, sizeof(double), 1, file);
    fread(&node.lon, sizeof(double), 1, file);
    return node;
}

static double aStar(uint32_t start_id, uint32_t goal_id,
    const std::vector<uint32_t>& ids,
    const std::vector<uint32_t>& offsets,
    FILE* graph_file,
    uint32_t coord_offset,
    uint32_t adj_offset,
    std::vector<double>& g_scores,
    std::vector<double>& f_scores,
    std::vector<uint32_t>& parents) {

    size_t start = binarySearch(ids, start_id);
    size_t goal = binarySearch(ids, goal_id);
    if (start == size_t(-1) || goal == size_t(-1)) return -1.0;

    std::fill(g_scores.begin(), g_scores.end(), -1.0);
    std::fill(f_scores.begin(), f_scores.end(), -1.0);

    std::priority_queue<PathEntry, std::vector<PathEntry>, std::greater<PathEntry>> pq;

    SearchNode goal_node = getNode(graph_file, coord_offset, goal);
    SearchNode start_node = getNode(graph_file, coord_offset, start);

    g_scores[start] = 0.0;
    f_scores[start] = haversine(start_node, goal_node);
    pq.push({ static_cast<uint32_t>(start), f_scores[start] });
    parents[start] = static_cast<uint32_t>(start);

    while (!pq.empty()) {
        uint32_t current = pq.top().node;
        double current_cost = pq.top().cost;
        pq.pop();

        if (current == goal) return g_scores[goal];
        if (current_cost > f_scores[current]) continue;

        uint32_t edge_start = (current == 0) ? 0 : offsets[current - 1];
        uint32_t edge_count = offsets[current] - edge_start;

        std::vector<uint32_t> neighbors(edge_count);
        fseek(graph_file, adj_offset + static_cast<long>(edge_start * sizeof(uint32_t)), SEEK_SET);
        fread(neighbors.data(), sizeof(uint32_t), edge_count, graph_file);

        SearchNode current_node = getNode(graph_file, coord_offset, current);

        for (uint32_t neighbor_id : neighbors) {
            size_t neighbor_idx = binarySearch(ids, neighbor_id);
            if (neighbor_idx == size_t(-1)) continue;

            SearchNode neighbor_node = getNode(graph_file, coord_offset, neighbor_idx);

            double edge_cost = haversine(current_node, neighbor_node);
            double tentative_g = g_scores[current] + edge_cost;

            if (g_scores[neighbor_idx] < 0 || tentative_g + 1e-9 < g_scores[neighbor_idx]) {
                g_scores[neighbor_idx] = tentative_g;
                f_scores[neighbor_idx] = tentative_g + haversine(neighbor_node, goal_node);
                parents[neighbor_idx] = current;
                pq.push({ static_cast<uint32_t>(neighbor_idx), f_scores[neighbor_idx] });
            }
        }
    }

    return g_scores[goal];
}

void run_search(FILE* graph_file, FILE* input_file, FILE* output_file, bool full_output) {
    fseek(graph_file, 0, SEEK_SET);

    size_t num_nodes = 0;
    fread(&num_nodes, sizeof(num_nodes), 1, graph_file);

    std::vector<uint32_t> ids(num_nodes);
    fread(ids.data(), sizeof(uint32_t), num_nodes, graph_file);

    uint32_t coord_offset = static_cast<uint32_t>(sizeof(size_t) + num_nodes * sizeof(uint32_t));
    uint32_t offset_offset = coord_offset + static_cast<uint32_t>(num_nodes * 2 * sizeof(double));

    fseek(graph_file, offset_offset, SEEK_SET);
    std::vector<uint32_t> offsets(num_nodes);
    fread(offsets.data(), sizeof(uint32_t), num_nodes, graph_file);

    uint32_t adj_offset = offset_offset + static_cast<uint32_t>(num_nodes * sizeof(uint32_t));

    std::vector<double> g_scores(num_nodes + 1, -1.0);
    std::vector<double> f_scores(num_nodes + 1, -1.0);
    std::vector<uint32_t> parents(num_nodes + 1);

    uint32_t src_id, dst_id;
    while (fscanf(input_file, "%u%u", &src_id, &dst_id) == 2) {
        // вход: 1..N, внутри: 0..N-1
        double distance = aStar(src_id - 1, dst_id - 1,
            ids, offsets,
            graph_file, coord_offset, adj_offset,
            g_scores, f_scores, parents);

        if (distance < 0) fprintf(output_file, "-1");
        else fprintf(output_file, "%.6f", distance);

        if (full_output) {
            if (distance < 0) {
                fprintf(output_file, " 0");
            }
            else {
                size_t current = binarySearch(ids, dst_id - 1);
                size_t start_idx = binarySearch(ids, src_id - 1);

                std::vector<uint32_t> path;
                path.push_back(ids[current]);

                while (current != start_idx) {
                    current = parents[current];
                    path.push_back(ids[current]);
                }

                fprintf(output_file, " %zu", path.size());
                for (int i = static_cast<int>(path.size()) - 1; i >= 0; --i) {
                    fprintf(output_file, " %u", path[i] + 1);
                }
            }
        }

        fprintf(output_file, "\n");
    }
}
