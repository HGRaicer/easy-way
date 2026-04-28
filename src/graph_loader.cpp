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

void load_routes_from_file(const std::string& result_txt, std::vector<Route>& routes_out) {
    static const std::array<Rgba, 10> kRoutePalette = { {
        {  74, 144, 226, 255},
        {  80, 190, 170, 255},
        { 156, 120, 210, 255},
        { 230, 145,  72, 255},
        { 105, 185, 110, 255},
        { 214, 105, 135, 255},
        {  88, 170, 205, 255},
        { 218, 190,  85, 255},
        { 170, 135, 100, 255},
        { 130, 160, 230, 255}
    } };

    std::ifstream in(result_txt);
    if (!in) return;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '-') continue;

        std::istringstream iss(line);

        Route r;
        std::uint32_t node_count = 0;

        // result.txt format, one route per line:
        // <time_seconds> <distance_meters> <node_count> <dense_id_1> ... <dense_id_N>
        if (!(iss >> r.total_time_s >> r.total_dist_m >> node_count)) continue;
        if (node_count < 2) continue;

        r.nodes.reserve(node_count);
        for (std::uint32_t i = 0; i < node_count; ++i) {
            std::uint32_t id = 0;
            if (!(iss >> id)) break;
            if (id > 0) {
                r.nodes.push_back(id - 1); // result.txt uses dense ids 1..N, app uses 0..N-1
            }
        }

        if (r.nodes.size() >= 2) {
            const std::size_t route_index = routes_out.size();
            r.label = "Route " + std::to_string(route_index + 1);
            r.color = kRoutePalette[route_index % kRoutePalette.size()];
            r.visible = true;
            routes_out.push_back(std::move(r));
        }
    }
}
