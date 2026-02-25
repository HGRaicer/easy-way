#include "preprocess.hpp"

#include <osmium/io/any_input.hpp>
#include <osmium/io/reader.hpp>
#include <osmium/visitor.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/handler.hpp>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <string_view>
#include <vector>

namespace fs = std::filesystem;

// -------------------- filters (как в osm_graph.cpp) --------------------
static bool is_drivable_highway(const char* highway) {
    if (!highway) return false;
    // MVP фильтр, можно расширять
    static const std::vector<std::string_view> ok = {
        "motorway","trunk","primary","secondary","tertiary",
        "unclassified","residential","service","living_street"
    };
    for (auto v : ok) if (v == highway) return true;
    return false;
}

static bool is_oneway(const osmium::TagList& tags) {
    const char* v = tags.get_value_by_key("oneway");
    if (!v) return false;
    std::string_view s(v);
    return (s == "yes" || s == "1" || s == "true");
}

// -------------------- binary records --------------------
struct EdgeOSM {
    std::int64_t from{};
    std::int64_t to{};
};

struct EdgeDense {
    uint32_t from{};
    uint32_t to{};
};

static bool write_all(std::ofstream& out, const void* data, size_t bytes) {
    out.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(bytes));
    return static_cast<bool>(out);
}

static bool read_record(std::ifstream& in, void* data, size_t bytes) {
    in.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(bytes));
    return static_cast<bool>(in);
}

// -------------------- external sort (fixed-size records) --------------------
template <typename T, typename Less>
static bool external_sort_unique(const fs::path& input,
    const fs::path& output,
    const fs::path& tmp_dir,
    size_t mem_bytes,
    Less less,
    std::string& err) {
    if (mem_bytes < sizeof(T) * 1024) mem_bytes = sizeof(T) * 1024;

    fs::create_directories(tmp_dir);

    // 1) read chunks -> sort -> unique -> write runs
    std::vector<fs::path> runs;
    {
        std::ifstream in(input, std::ios::binary);
        if (!in) { err = "Can't open temp input for sort: " + input.string(); return false; }

        const size_t max_items = mem_bytes / sizeof(T);
        std::vector<T> buf;
        buf.reserve(max_items);

        size_t run_id = 0;
        while (true) {
            buf.clear();
            T x{};
            for (size_t i = 0; i < max_items; ++i) {
                if (!read_record(in, &x, sizeof(T))) break;
                buf.push_back(x);
            }
            if (buf.empty()) break;

            std::sort(buf.begin(), buf.end(), less);

            // unique within chunk
            auto eq = [&](const T& a, const T& b) {
                return !less(a, b) && !less(b, a);
                };
            buf.erase(std::unique(buf.begin(), buf.end(), eq), buf.end());

            fs::path run_path = tmp_dir / ("run_" + std::to_string(run_id++) + ".bin");
            std::ofstream out(run_path, std::ios::binary);
            if (!out) { err = "Can't create run file: " + run_path.string(); return false; }

            out.write(reinterpret_cast<const char*>(buf.data()),
                static_cast<std::streamsize>(buf.size() * sizeof(T)));
            if (!out) { err = "Write failed for run file: " + run_path.string(); return false; }

            runs.push_back(run_path);

            if (!in) break; // EOF
        }
    }

    if (runs.empty()) {
        // empty input -> empty output
        std::ofstream out(output, std::ios::binary);
        return static_cast<bool>(out);
    }

    if (runs.size() == 1) {
        // just move
        std::error_code ec;
        fs::rename(runs[0], output, ec);
        if (ec) {
            // fallback copy
            std::ifstream src(runs[0], std::ios::binary);
            std::ofstream dst(output, std::ios::binary);
            dst << src.rdbuf();
            src.close();
            dst.close();
            fs::remove(runs[0], ec);
        }
        return true;
    }

    // 2) k-way merge with unique
    struct Item {
        T value{};
        size_t run_idx{};
    };
    struct PQCmp {
        Less less;
        bool operator()(const Item& a, const Item& b) const {
            // priority_queue is max-heap; invert to get min-heap
            return less(b.value, a.value);
        }
    };

    std::vector<std::ifstream> ins;
    ins.reserve(runs.size());
    for (auto& rp : runs) {
        ins.emplace_back(rp, std::ios::binary);
        if (!ins.back()) { err = "Can't open run file: " + rp.string(); return false; }
    }

    std::priority_queue<Item, std::vector<Item>, PQCmp> pq{ PQCmp{less} };
    for (size_t i = 0; i < ins.size(); ++i) {
        T v{};
        if (read_record(ins[i], &v, sizeof(T))) {
            pq.push(Item{ v, i });
        }
    }

    std::ofstream out(output, std::ios::binary);
    if (!out) { err = "Can't open output for merge: " + output.string(); return false; }

    bool have_last = false;
    T last{};

    auto eq = [&](const T& a, const T& b) {
        return !less(a, b) && !less(b, a);
        };

    while (!pq.empty()) {
        Item it = pq.top();
        pq.pop();

        if (!have_last || !eq(it.value, last)) {
            out.write(reinterpret_cast<const char*>(&it.value), sizeof(T));
            if (!out) { err = "Write failed during merge: " + output.string(); return false; }
            last = it.value;
            have_last = true;
        }

        T next{};
        if (read_record(ins[it.run_idx], &next, sizeof(T))) {
            pq.push(Item{ next, it.run_idx });
        }
    }

    // cleanup runs
    std::error_code ec;
    for (auto& rp : runs) fs::remove(rp, ec);

    return true;
}

static bool map_osm_to_dense(const std::vector<std::int64_t>& osm_ids, std::int64_t osm_id, uint32_t& out_idx) {
    auto it = std::lower_bound(osm_ids.begin(), osm_ids.end(), osm_id);
    if (it == osm_ids.end() || *it != osm_id) return false;
    out_idx = static_cast<uint32_t>(std::distance(osm_ids.begin(), it));
    return true;
}

// -------------------- pass 1: ways -> tmp_nodes + tmp_edges_osm --------------------
static bool pass_ways_collect(const std::string& pbf_path,
    const fs::path& tmp_nodes,
    const fs::path& tmp_edges,
    std::string& err) {
    std::ofstream nodes_out(tmp_nodes, std::ios::binary);
    if (!nodes_out) { err = "Can't create tmp_nodes: " + tmp_nodes.string(); return false; }

    std::ofstream edges_out(tmp_edges, std::ios::binary);
    if (!edges_out) { err = "Can't create tmp_edges: " + tmp_edges.string(); return false; }

    struct Handler : public osmium::handler::Handler {
        std::ofstream& nodes_out;
        std::ofstream& edges_out;

        Handler(std::ofstream& n, std::ofstream& e) : nodes_out(n), edges_out(e) {}

        void way(const osmium::Way& w) {
            const char* highway = w.tags().get_value_by_key("highway");
            if (!is_drivable_highway(highway)) return;

            const bool oneway = is_oneway(w.tags());
            const auto& nr = w.nodes();
            if (nr.size() < 2) return;

            // записываем node_id (для списка уникальных узлов)
            for (const auto& nref : nr) {
                std::int64_t id = static_cast<std::int64_t>(nref.ref());
                nodes_out.write(reinterpret_cast<const char*>(&id), sizeof(id));
            }

            // записываем ребра по сегментам
            for (size_t i = 1; i < nr.size(); ++i) {
                std::int64_t a = static_cast<std::int64_t>(nr[i - 1].ref());
                std::int64_t b = static_cast<std::int64_t>(nr[i].ref());
                EdgeOSM e1{ a, b };
                edges_out.write(reinterpret_cast<const char*>(&e1), sizeof(e1));
                if (!oneway) {
                    EdgeOSM e2{ b, a };
                    edges_out.write(reinterpret_cast<const char*>(&e2), sizeof(e2));
                }
            }
        }
    };

    try {
        osmium::io::Reader reader{ pbf_path, osmium::osm_entity_bits::way };
        Handler h{ nodes_out, edges_out };
        osmium::apply(reader, h);
        reader.close();
    }
    catch (const std::exception& e) {
        err = std::string("WAYS pass failed: ") + e.what();
        return false;
    }

    if (!nodes_out || !edges_out) {
        err = "Write failed during WAYS pass (tmp files).";
        return false;
    }
    return true;
}

// -------------------- pass 2: nodes -> fill coords for needed osm_ids --------------------
static bool pass_nodes_fill_coords(const std::string& pbf_path,
    const std::vector<std::int64_t>& osm_ids,
    std::vector<double>& lat,
    std::vector<double>& lon,
    std::string& err) {
    lat.assign(osm_ids.size(), 0.0);
    lon.assign(osm_ids.size(), 0.0);
    std::vector<uint8_t> have(osm_ids.size(), 0);

    struct Handler : public osmium::handler::Handler {
        const std::vector<std::int64_t>& osm_ids;
        std::vector<double>& lat;
        std::vector<double>& lon;
        std::vector<uint8_t>& have;

        Handler(const std::vector<std::int64_t>& ids,
            std::vector<double>& la,
            std::vector<double>& lo,
            std::vector<uint8_t>& hv)
            : osm_ids(ids), lat(la), lon(lo), have(hv) {
        }

        void node(const osmium::Node& n) {
            const auto loc = n.location();
            if (!loc.valid()) return;

            std::int64_t id = static_cast<std::int64_t>(n.id());
            auto it = std::lower_bound(osm_ids.begin(), osm_ids.end(), id);
            if (it == osm_ids.end() || *it != id) return;

            const uint32_t idx = static_cast<uint32_t>(std::distance(osm_ids.begin(), it));
            if (!have[idx]) {
                lat[idx] = loc.lat();
                lon[idx] = loc.lon();
                have[idx] = 1;
            }
        }
    };

    try {
        osmium::io::Reader reader{ pbf_path, osmium::osm_entity_bits::node };
        Handler h{ osm_ids, lat, lon, have };
        osmium::apply(reader, h);
        reader.close();
    }
    catch (const std::exception& e) {
        err = std::string("NODES pass failed: ") + e.what();
        return false;
    }

    size_t missing = 0;
    for (auto v : have) if (!v) ++missing;
    if (missing != 0) {
        err = "Missing coordinates for " + std::to_string(missing) + " required nodes.";
        return false;
    }

    return true;
}

// -------------------- edges osm -> dense (tmp) --------------------
static bool convert_edges_to_dense(const fs::path& tmp_edges_osm,
    const std::vector<std::int64_t>& osm_ids,
    const fs::path& tmp_edges_dense,
    std::string& err) {
    std::ifstream in(tmp_edges_osm, std::ios::binary);
    if (!in) { err = "Can't open tmp_edges_osm: " + tmp_edges_osm.string(); return false; }

    std::ofstream out(tmp_edges_dense, std::ios::binary);
    if (!out) { err = "Can't create tmp_edges_dense: " + tmp_edges_dense.string(); return false; }

    EdgeOSM e{};
    while (read_record(in, &e, sizeof(e))) {
        uint32_t a, b;
        if (!map_osm_to_dense(osm_ids, e.from, a)) continue;
        if (!map_osm_to_dense(osm_ids, e.to, b)) continue;
        EdgeDense d{ a, b };
        out.write(reinterpret_cast<const char*>(&d), sizeof(d));
    }

    if (!out) { err = "Write failed for tmp_edges_dense."; return false; }
    return true;
}

// -------------------- build graph.bin from sorted unique dense edges --------------------
static bool build_bin_from_sorted_edges(const fs::path& edges_sorted,
    const std::vector<std::int64_t>& osm_ids,
    const std::vector<double>& lat,
    const std::vector<double>& lon,
    const std::string& out_bin_path,
    std::string& err) {
    const size_t n = osm_ids.size();
    if (n == 0) { err = "0 nodes after filtering."; return false; }

    // degree
    std::vector<uint32_t> degree(n, 0);
    {
        std::ifstream in(edges_sorted, std::ios::binary);
        if (!in) { err = "Can't open edges_sorted: " + edges_sorted.string(); return false; }
        EdgeDense e{};
        while (read_record(in, &e, sizeof(e))) {
            if (e.from >= n) continue;
            if (degree[e.from] == std::numeric_limits<uint32_t>::max()) {
                err = "Degree overflow for node " + std::to_string(e.from);
                return false;
            }
            degree[e.from]++;
        }
    }

    // offsets prefix sum
    std::vector<uint32_t> offsets(n, 0);
    uint64_t total = 0;
    for (size_t i = 0; i < n; ++i) {
        total += degree[i];
        if (total > std::numeric_limits<uint32_t>::max()) {
            err = "Too many edges for uint32 offsets in this format.";
            return false;
        }
        offsets[i] = static_cast<uint32_t>(total);
    }

    // write graph.bin
    std::ofstream out(out_bin_path, std::ios::binary);
    if (!out) { err = "Can't open output bin: " + out_bin_path; return false; }

    // 1) size_t N
    out.write(reinterpret_cast<const char*>(&n), sizeof(size_t));

    // 2) ids: 0..N-1 (как ожидает search.cpp)
    for (uint32_t i = 0; i < static_cast<uint32_t>(n); ++i) {
        out.write(reinterpret_cast<const char*>(&i), sizeof(uint32_t));
    }

    // 3) coords: lat, lon as double (как ты просил)
    for (uint32_t i = 0; i < static_cast<uint32_t>(n); ++i) {
        out.write(reinterpret_cast<const char*>(&lat[i]), sizeof(double));
        out.write(reinterpret_cast<const char*>(&lon[i]), sizeof(double));
    }

    // 4) offsets
    out.write(reinterpret_cast<const char*>(offsets.data()), static_cast<std::streamsize>(n * sizeof(uint32_t)));

    // 5) adjacency (уже отсортировано по from,to, поэтому пишем последовательно)
    {
        std::ifstream in(edges_sorted, std::ios::binary);
        if (!in) { err = "Can't reopen edges_sorted: " + edges_sorted.string(); return false; }

        EdgeDense e{};
        while (read_record(in, &e, sizeof(e))) {
            out.write(reinterpret_cast<const char*>(&e.to), sizeof(uint32_t));
        }
    }

    if (!out) { err = "Write failed for output bin: " + out_bin_path; return false; }
    return true;
}

// -------------------- public entry --------------------
bool preprocess_pbf_to_bin(const std::string& pbf_path,
    const std::string& out_bin_path,
    const std::string& out_map_csv_path,
    std::string& err) {
    try {
        // temp directory рядом с output (чтобы не было медленного cross-disk)
        fs::path outp = fs::absolute(out_bin_path);
        fs::path base_dir = outp.parent_path();
        fs::path tmp_dir = base_dir / (outp.filename().string() + ".tmp");

        fs::create_directories(tmp_dir);

        const fs::path tmp_nodes_raw = tmp_dir / "nodes_raw.bin";
        const fs::path tmp_edges_osm = tmp_dir / "edges_osm.bin";
        const fs::path tmp_nodes_sorted = tmp_dir / "nodes_sorted_unique.bin";
        const fs::path tmp_edges_dense = tmp_dir / "edges_dense.bin";
        const fs::path tmp_edges_sorted = tmp_dir / "edges_dense_sorted_unique.bin";

        // лимит памяти на внешнюю сортировку (можно поднять)
        const size_t MEM_BYTES = 256ull * 1024ull * 1024ull; // 256 MB

        // Pass 1: ways -> raw temp files
        if (!pass_ways_collect(pbf_path, tmp_nodes_raw, tmp_edges_osm, err)) return false;

        // Sort unique node ids (external)
        if (!external_sort_unique<std::int64_t>(
            tmp_nodes_raw, tmp_nodes_sorted, tmp_dir / "runs_nodes",
            MEM_BYTES, std::less<std::int64_t>{}, err)) return false;

        // Load osm_ids into RAM (compact vector)
        std::vector<std::int64_t> osm_ids;
        {
            std::ifstream in(tmp_nodes_sorted, std::ios::binary);
            if (!in) { err = "Can't open nodes_sorted_unique: " + tmp_nodes_sorted.string(); return false; }
            std::int64_t id{};
            while (read_record(in, &id, sizeof(id))) osm_ids.push_back(id);
        }
        if (osm_ids.empty()) {
            err = "0 nodes after filtering ways (check highway filter or PBF).";
            return false;
        }

        // Pass 2: nodes -> coords for required ids
        std::vector<double> lat, lon;
        if (!pass_nodes_fill_coords(pbf_path, osm_ids, lat, lon, err)) return false;

        // Convert edges osm -> dense (tmp)
        if (!convert_edges_to_dense(tmp_edges_osm, osm_ids, tmp_edges_dense, err)) return false;

        // Sort unique edges by (from,to) (external)
        auto edge_less = [](const EdgeDense& a, const EdgeDense& b) {
            if (a.from != b.from) return a.from < b.from;
            return a.to < b.to;
            };
        if (!external_sort_unique<EdgeDense>(
            tmp_edges_dense, tmp_edges_sorted, tmp_dir / "runs_edges",
            MEM_BYTES, edge_less, err)) return false;

        // Build graph.bin from sorted edges (CSR) with coords double
        if (!build_bin_from_sorted_edges(tmp_edges_sorted, osm_ids, lat, lon, out_bin_path, err)) return false;

        // Write mapping CSV (dense_id -> osm_id)
        if (!out_map_csv_path.empty()) {
            std::ofstream csv(out_map_csv_path);
            if (!csv) { err = "Can't open map csv: " + out_map_csv_path; return false; }
            csv << "dense_id,osm_id,lat,lon\n";
            for (uint32_t i = 0; i < static_cast<uint32_t>(osm_ids.size()); ++i) {
                csv << (i + 1) << "," << osm_ids[i] << "," << lat[i] << "," << lon[i] << "\n";
            }
        }

        // cleanup temp
        std::error_code ec;
        fs::remove_all(tmp_dir, ec);
        return true;
    }
    catch (const std::exception& e) {
        err = std::string("Preprocess exception: ") + e.what();
        return false;
    }
}