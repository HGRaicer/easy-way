#include "search.h"

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>
#include <cstdint>
#include <cstring>

constexpr double PI = 3.141592653589793;
constexpr double EARTH_RADIUS = 6371000.0;

// For time heuristic: should be >= any modeled road speed. 60 m/s ≈ 216 km/h (safe)
constexpr double VMAX_MPS = 60.0;

struct NodeCoord {
    double lat{};
    double lon{};
};

struct PQItem {
    uint32_t node{};
    double f{};
    bool operator>(const PQItem& o) const noexcept {
        return f > o.f || (f == o.f && node > o.node);
    }
};

static double haversine(const NodeCoord& a, const NodeCoord& b) noexcept {
    const double dlat = (b.lat - a.lat) * PI / 180.0;
    const double dlon = (b.lon - a.lon) * PI / 180.0;

    const double lat1 = a.lat * PI / 180.0;
    const double lat2 = b.lat * PI / 180.0;

    const double s1 = std::sin(dlat * 0.5);
    const double s2 = std::sin(dlon * 0.5);

    const double x = s1 * s1 + std::cos(lat1) * std::cos(lat2) * s2 * s2;
    return 2.0 * EARTH_RADIUS * std::asin(std::sqrt(x));
}

static long long tell64(FILE* f) {
#if defined(_WIN32)
    return _ftelli64(f);
#else
    return ftell(f);
#endif
}

static int seek64(FILE* f, long long off, int whence) {
#if defined(_WIN32)
    return _fseeki64(f, off, whence);
#else
    return fseek(f, off, whence);
#endif
}

// graph.bin v2 light:
// magic[4]="GBIN", version(u32)=2, N(u64), flags(u32 bit0=speed_present)
// then: ids[u32*N], coords[double*2N], offsets[u32*N], adjacency[u32*M], speed_mps[float*M]
struct GraphHeaderV2 {
    uint32_t version{};
    uint64_t N{};
    uint32_t flags{};
};

static bool read_header_v2(FILE* f, GraphHeaderV2& h) {
    unsigned char magic[4];
    if (fread(magic, 1, 4, f) != 4) return false;
    if (std::memcmp(magic, "GBIN", 4) != 0) return false;

    if (fread(&h.version, sizeof(uint32_t), 1, f) != 1) return false;
    if (h.version != 2) return false;

    if (fread(&h.N, sizeof(uint64_t), 1, f) != 1) return false;
    if (fread(&h.flags, sizeof(uint32_t), 1, f) != 1) return false;

    return true;
}

void run_search(FILE* graph_file,
    FILE* input_file,
    FILE* output_file,
    bool full_output,
    SearchMetric metric) {
    if (!graph_file || !input_file || !output_file) return;

    seek64(graph_file, 0, SEEK_SET);

    GraphHeaderV2 h2{};
    const long long start_pos = tell64(graph_file);
    const bool is_v2 = read_header_v2(graph_file, h2);

    uint64_t N64 = 0;
    uint32_t flags = 0;
    long long base_after_header = 0;

    if (is_v2) {
        N64 = h2.N;
        flags = h2.flags;
        base_after_header = tell64(graph_file);
    }
    else {
        // old format fallback: supports only distance and has no speed array
        if (metric == SearchMetric::Time) {
            fprintf(output_file, "ERROR: old graph.bin supports only distance. Re-run preprocess.\n");
            return;
        }
        seek64(graph_file, start_pos, SEEK_SET);

        size_t n_old = 0;
        if (fread(&n_old, sizeof(size_t), 1, graph_file) != 1) {
            fprintf(output_file, "ERROR: can't read old header.\n");
            return;
        }
        N64 = static_cast<uint64_t>(n_old);
        flags = 0;
        base_after_header = static_cast<long long>(sizeof(size_t));
    }

    if (N64 == 0 || N64 > std::numeric_limits<uint32_t>::max()) {
        fprintf(output_file, "ERROR: invalid N\n");
        return;
    }
    const uint32_t N = static_cast<uint32_t>(N64);

    // ids
    std::vector<uint32_t> ids(N);
    fread(ids.data(), sizeof(uint32_t), N, graph_file);

    // coords
    const long long coord_offset = base_after_header + static_cast<long long>(N) * sizeof(uint32_t);
    seek64(graph_file, coord_offset, SEEK_SET);

    std::vector<NodeCoord> coords(N);
    for (uint32_t i = 0; i < N; ++i) {
        fread(&coords[i].lat, sizeof(double), 1, graph_file);
        fread(&coords[i].lon, sizeof(double), 1, graph_file);
    }

    // offsets
    const long long offsets_offset = coord_offset + static_cast<long long>(N) * 2LL * sizeof(double);
    seek64(graph_file, offsets_offset, SEEK_SET);

    std::vector<uint32_t> offsets(N);
    fread(offsets.data(), sizeof(uint32_t), N, graph_file);

    const uint32_t M = offsets[N - 1];
    const long long adj_offset = offsets_offset + static_cast<long long>(N) * sizeof(uint32_t);

    const bool speed_present = is_v2 && ((flags & 1u) != 0u);
    const long long speed_offset = speed_present
        ? (adj_offset + static_cast<long long>(M) * sizeof(uint32_t))
        : 0;

    if (metric == SearchMetric::Time && !speed_present) {
        fprintf(output_file, "ERROR: graph.bin has no speed array. Re-run preprocess.\n");
        return;
    }

    // A* buffers
    std::vector<double> g(N, std::numeric_limits<double>::infinity());
    std::vector<double> f(N, std::numeric_limits<double>::infinity());
    std::vector<uint32_t> parent(N, 0);
    std::vector<float> parent_speed(N, 1.0f); // speed used to reach node from parent

    auto heuristic = [&](uint32_t u, uint32_t goal) -> double {
        const double d = haversine(coords[u], coords[goal]);
        if (metric == SearchMetric::Distance) return d;         // meters
        return d / VMAX_MPS;                                    // seconds
        };

    auto read_neighbors = [&](uint32_t u, std::vector<uint32_t>& neigh, std::vector<float>& speed) {
        const uint32_t start = (u == 0) ? 0 : offsets[u - 1];
        const uint32_t end = offsets[u];
        const uint32_t cnt = end - start;

        neigh.resize(cnt);
        seek64(graph_file, adj_offset + static_cast<long long>(start) * sizeof(uint32_t), SEEK_SET);
        fread(neigh.data(), sizeof(uint32_t), cnt, graph_file);

        speed.resize(cnt);
        if (speed_present) {
            seek64(graph_file, speed_offset + static_cast<long long>(start) * sizeof(float), SEEK_SET);
            fread(speed.data(), sizeof(float), cnt, graph_file);
        }
        else {
            for (uint32_t i = 0; i < cnt; ++i) speed[i] = 1.0f;
        }
        };

    // Input pairs: 1..N
    uint32_t src1 = 0, dst1 = 0;
    while (fscanf(input_file, "%u%u", &src1, &dst1) == 2) {
        if (src1 == 0 || dst1 == 0 || src1 > N || dst1 > N) {
            // time dist [k path...]
            fprintf(output_file, "-1 -1");
            if (full_output) fprintf(output_file, " 0\n");
            else fprintf(output_file, "\n");
            continue;
        }

        const uint32_t start = src1 - 1;
        const uint32_t goal = dst1 - 1;

        std::fill(g.begin(), g.end(), std::numeric_limits<double>::infinity());
        std::fill(f.begin(), f.end(), std::numeric_limits<double>::infinity());
        for (uint32_t i = 0; i < N; ++i) {
            parent[i] = i;
            parent_speed[i] = 1.0f;
        }

        std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;

        g[start] = 0.0;
        f[start] = heuristic(start, goal);
        pq.push({ start, f[start] });

        bool found = false;
        std::vector<uint32_t> neigh;
        std::vector<float> speed;

        while (!pq.empty()) {
            const auto cur = pq.top();
            pq.pop();

            const uint32_t u = cur.node;
            if (cur.f > f[u]) continue;

            if (u == goal) {
                found = true;
                break;
            }

            read_neighbors(u, neigh, speed);

            for (size_t i = 0; i < neigh.size(); ++i) {
                const uint32_t v = neigh[i];

                const double dist_m = haversine(coords[u], coords[v]);
                const double sp = std::max(0.1f, speed[i]);

                double edge_cost = 0.0;
                if (metric == SearchMetric::Distance) edge_cost = dist_m;
                else edge_cost = dist_m / sp;

                const double cand = g[u] + edge_cost;
                if (cand + 1e-12 < g[v]) {
                    g[v] = cand;
                    f[v] = cand + heuristic(v, goal);
                    parent[v] = u;
                    parent_speed[v] = static_cast<float>(sp); // remember speed for this parent edge
                    pq.push({ v, f[v] });
                }
            }
        }

        if (!found || !std::isfinite(g[goal])) {
            fprintf(output_file, "-1 -1");
            if (full_output) fprintf(output_file, " 0\n");
            else fprintf(output_file, "\n");
            continue;
        }

        // Reconstruct path ALWAYS (we need it to compute both time and distance)
        std::vector<uint32_t> path;
        {
            uint32_t cur = goal;
            path.push_back(cur);
            while (cur != start) {
                const uint32_t p = parent[cur];
                if (p == cur) break; // safety
                cur = p;
                path.push_back(cur);
            }
            std::reverse(path.begin(), path.end());
        }

        // Compute both metrics on this path
        double total_dist_m = 0.0;
        double total_time_s = 0.0;

        if (path.size() >= 2) {
            for (size_t i = 1; i < path.size(); ++i) {
                const uint32_t u = path[i - 1];
                const uint32_t v = path[i];
                const double d = haversine(coords[u], coords[v]);
                total_dist_m += d;

                // speed for node v is the edge (parent[v] -> v)
                const double sp = std::max(0.1f, parent_speed[v]);
                total_time_s += d / sp;
            }
        }

        // OUTPUT: time first, then distance
        fprintf(output_file, "%.6f %.6f", total_time_s, total_dist_m);

        if (full_output) {
            fprintf(output_file, " %zu", path.size());
            for (uint32_t v : path) {
                fprintf(output_file, " %u", v + 1); // back to 1..N
            }
        }

        fprintf(output_file, "\n");
    }
}