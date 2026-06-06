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


constexpr double VMAX_MPS = 30.5;

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

struct BackwardEdge {
    uint32_t from{};
    float speed{};
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

    // Загрузка ID вершин
    std::vector<uint32_t> ids(N);
    fread(ids.data(), sizeof(uint32_t), N, graph_file);

    // Загрузка координат
    const long long coord_offset = base_after_header + static_cast<long long>(N) * sizeof(uint32_t);
    seek64(graph_file, coord_offset, SEEK_SET);

    std::vector<NodeCoord> coords(N);
    for (uint32_t i = 0; i < N; ++i) {
        fread(&coords[i].lat, sizeof(double), 1, graph_file);
        fread(&coords[i].lon, sizeof(double), 1, graph_file);
    }

    // Загрузка смещений (offsets)
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

    // КРИТИЧЕСКОЕ УСКОРЕНИЕ: Считываем массивы смежности и скоростей ЦЕЛИКОМ в память
    std::vector<uint32_t> adjacency(M);
    seek64(graph_file, adj_offset, SEEK_SET);
    fread(adjacency.data(), sizeof(uint32_t), M, graph_file);

    std::vector<float> speed_mps(M, 1.0f);
    if (speed_present) {
        seek64(graph_file, speed_offset, SEEK_SET);
        fread(speed_mps.data(), sizeof(float), M, graph_file);
    }

    // ПОСТРОЕНИЕ ОБРАТНОГО ГРАФА ДЛЯ BACKWARD SEARCH
    std::vector<std::vector<BackwardEdge>> back_edges(N);
    for (uint32_t u = 0; u < N; ++u) {
        const uint32_t start_idx = (u == 0) ? 0 : offsets[u - 1];
        const uint32_t end_idx = offsets[u];
        for (uint32_t i = start_idx; i < end_idx; ++i) {
            uint32_t v = adjacency[i];
            float sp = speed_present ? speed_mps[i] : 1.0f;
            back_edges[v].push_back({ u, sp });
        }
    }

    // Буферы для двунаправленного поиска (вынесены из цикла для избежания реаллокаций)
    std::vector<double> g_f(N, std::numeric_limits<double>::infinity());
    std::vector<double> g_b(N, std::numeric_limits<double>::infinity());
    std::vector<uint32_t> parent_f(N, 0);
    std::vector<uint32_t> parent_b(N, 0);
    
    // Оптимизация O(1) сброса: query_id вместо тяжелого std::fill
    std::vector<uint32_t> query_f(N, 0);
    std::vector<uint32_t> query_b(N, 0);
    uint32_t current_query = 0;

    // Лямбда для вычисления базовой географической эвристики
    auto h_dist = [&](uint32_t u, uint32_t v) -> double {
        const double d = haversine(coords[u], coords[v]);
        if (metric == SearchMetric::Distance) return d;
        return d / VMAX_MPS;
    };

    fprintf(output_file, "=== A* Optimization Result ===\n");
    uint32_t src1 = 0, dst1 = 0;

    while (fscanf(input_file, "%u%u", &src1, &dst1) == 2) {
        if (src1 == 0 || dst1 == 0 || src1 > N || dst1 > N) {
            fprintf(output_file, "-1 -1");
            if (full_output) fprintf(output_file, " 0\n");
            else fprintf(output_file, "\n");
            continue;
        }

        const uint32_t start = src1 - 1;
        const uint32_t goal = dst1 - 1;

        if (start == goal) {
            fprintf(output_file, "0.000000 0.000000");
            if (full_output) fprintf(output_file, " 1 %u\n", start + 1);
            else fprintf(output_file, "\n");
            continue;
        }

        current_query++;

        // Лямбды для быстрого доступа к g-scores с ленивым сбросом
        auto get_g_f = [&](uint32_t node) {
            return (query_f[node] == current_query) ? g_f[node] : std::numeric_limits<double>::infinity();
        };
        auto set_g_f = [&](uint32_t node, double val) {
            g_f[node] = val;
            query_f[node] = current_query;
        };
        auto get_g_b = [&](uint32_t node) {
            return (query_b[node] == current_query) ? g_b[node] : std::numeric_limits<double>::infinity();
        };
        auto set_g_b = [&](uint32_t node, double val) {
            g_b[node] = val;
            query_b[node] = current_query;
        };

        std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq_f;
        std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq_b;

        set_g_f(start, 0.0);
        set_g_b(goal, 0.0);
        parent_f[start] = start;
        parent_b[goal] = goal;

        // Потенциалы Икеды для двунаправленной симметрии А*
        // p(u) = 0.5 * (h(u, goal) - h(start, u))
        double p_start = 0.5 * (h_dist(start, goal) - 0.0);
        double p_goal = 0.5 * (0.0 - h_dist(start, goal));

        pq_f.push({ start, 0.0 + p_start });
        pq_b.push({ goal, 0.0 - p_goal });

        double best_cost = std::numeric_limits<double>::infinity();
        int32_t meeting_node = -1;

        while (!pq_f.empty() && !pq_b.empty()) {
            // Условие гарантированной остановки двунаправленного А*
            if (pq_f.top().f + pq_b.top().f >= best_cost) {
                break;
            }

            // Балансировка: развиваем то направление, у которого ключ в приоритетной очереди меньше
            if (pq_f.top().f <= pq_b.top().f) {
                // ВПЕРЕД
                auto curr = pq_f.top();
                pq_f.pop();
                uint32_t u = curr.node;

                double current_p = 0.5 * (h_dist(u, goal) - h_dist(start, u));
                if (curr.f > get_g_f(u) + current_p) continue;

                const uint32_t start_idx = (u == 0) ? 0 : offsets[u - 1];
                const uint32_t end_idx = offsets[u];

                for (uint32_t i = start_idx; i < end_idx; ++i) {
                    uint32_t v = adjacency[i];
                    double dist_m = haversine(coords[u], coords[v]);
                    double sp = speed_present ? std::max(0.1f, speed_mps[i]) : 1.0f;
                    double edge_cost = (metric == SearchMetric::Distance) ? dist_m : (dist_m / sp);

                    double cand = get_g_f(u) + edge_cost;
                    if (cand < get_g_f(v)) {
                        set_g_f(v, cand);
                        parent_f[v] = u;
                        double next_p = 0.5 * (h_dist(v, goal) - h_dist(start, v));
                        pq_f.push({ v, cand + next_p });

                        // Проверяем встречу путей
                        if (query_b[v] == current_query) {
                            double total_c = cand + g_b[v];
                            if (total_c < best_cost) {
                                best_cost = total_c;
                                meeting_node = v;
                            }
                        }
                    }
                }
            }
            else {
                // НАЗАД
                auto curr = pq_b.top();
                pq_b.pop();
                uint32_t v = curr.node;

                double current_p = 0.5 * (h_dist(v, goal) - h_dist(start, v));
                if (curr.f > get_g_b(v) - current_p) continue;

                for (const auto& edge : back_edges[v]) {
                    uint32_t u = edge.from; // Ребро идет из u в v
                    double dist_m = haversine(coords[u], coords[v]);
                    double sp = std::max(0.1f, edge.speed);
                    double edge_cost = (metric == SearchMetric::Distance) ? dist_m : (dist_m / sp);

                    double cand = get_g_b(v) + edge_cost;
                    if (cand < get_g_b(u)) {
                        set_g_b(u, cand);
                        parent_b[u] = v;
                        double next_p = 0.5 * (h_dist(u, goal) - h_dist(start, u));
                        pq_b.push({ u, cand - next_p });

                        // Проверяем встречу путей
                        if (query_f[u] == current_query) {
                            double total_c = g_f[u] + cand;
                            if (total_c < best_cost) {
                                best_cost = total_c;
                                meeting_node = u;
                            }
                        }
                    }
                }
            }
        }

        if (meeting_node == -1 || !std::isfinite(best_cost)) {
            fprintf(output_file, "-1 -1");
            if (full_output) fprintf(output_file, " 0\n");
            else fprintf(output_file, "\n");
            continue;
        }

        // Восстановление полного пути через точку встречи
        std::vector<uint32_t> path;
        {
            uint32_t cur = meeting_node;
            path.push_back(cur);
            while (cur != start) {
                cur = parent_f[cur];
                path.push_back(cur);
            }
            std::reverse(path.begin(), path.end());

            cur = meeting_node;
            while (cur != goal) {
                cur = parent_b[cur];
                path.push_back(cur);
            }
        }

        // Финальный точный расчет метрик на базе статических данных в RAM
        double total_dist_m = 0.0;
        double total_time_s = 0.0;

        if (path.size() >= 2) {
            for (size_t i = 1; i < path.size(); ++i) {
                const uint32_t u = path[i - 1];
                const uint32_t v = path[i];
                const double d = haversine(coords[u], coords[v]);
                total_dist_m += d;

                // Находим ребро в прямой структуре смежности для получения точной скорости
                const uint32_t s_idx = (u == 0) ? 0 : offsets[u - 1];
                const uint32_t e_idx = offsets[u];
                float edge_sp = 1.0f;
                for (uint32_t k = s_idx; k < e_idx; ++k) {
                    if (adjacency[k] == v) {
                        edge_sp = speed_present ? speed_mps[k] : 1.0f;
                        break;
                    }
                }
                const double sp = std::max(0.1f, edge_sp);
                total_time_s += d / sp;
            }
        }

        // Вывод результатов (форматирование строго соответствует оригиналу)
        fprintf(output_file, "%.6f %.6f", total_time_s, total_dist_m);

        if (full_output) {
            fprintf(output_file, " %zu", path.size());
            for (uint32_t v : path) {
                fprintf(output_file, " %u", v + 1); // Возврат к пользовательской индексации 1..N
            }
        }
        fprintf(output_file, "\n");
    }
}