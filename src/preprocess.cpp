#include "preprocess.hpp"

#include <osmium/io/reader.hpp>
#include <osmium/visitor.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/handler.hpp>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <queue>
#include <string>
#include <string_view>
#include <vector>
#include <limits>
#include <unordered_set>

namespace fs = std::filesystem;

// -------- speed model --------
static int parse_leading_int(std::string_view s) {
    int val = 0;
    bool any = false;
    for (char c : s) {
        if (c >= '0' && c <= '9') {
            any = true;
            val = val * 10 + (c - '0');
        }
        else if (any) {
            break;
        }
    }
    return any ? val : -1;
}

static int default_speed_kph(std::string_view highway) {
    if (highway == "motorway") return 110;
    if (highway == "motorway_link") return 70;

    if (highway == "trunk") return 90;
    if (highway == "trunk_link") return 60;

    if (highway == "primary") return 80;
    if (highway == "primary_link") return 50;

    if (highway == "secondary") return 70;
    if (highway == "secondary_link") return 45;

    if (highway == "tertiary") return 60;
    if (highway == "tertiary_link") return 40;

    if (highway == "residential") return 50;
    if (highway == "unclassified") return 50;
    if (highway == "road") return 40;
    if (highway == "living_street") return 10;
    if (highway == "service") return 20;

    return 50;
}

static bool is_drivable_highway(const char* highway) {
    if (!highway) return false;
    static const std::unordered_set<std::string_view> ok = {
        "motorway", "motorway_link",
        "trunk", "trunk_link",
        "primary", "primary_link",
        "secondary", "secondary_link",
        "tertiary", "tertiary_link",
        "unclassified", "residential",
        "service", "living_street",
        "road"
    };
    return ok.count(highway) > 0;
}

// oneway: 0 two-way, 1 forward, -1 reverse
static int oneway_dir(const osmium::TagList& tags) {
    const char* v = tags.get_value_by_key("oneway");
    if (!v) return 0;
    std::string_view s(v);
    if (s == "yes" || s == "1" || s == "true") return 1;
    if (s == "-1") return -1;
    return 0;
}

static float speed_mps_for_way(const osmium::Way& w) {
    const char* highway_c = w.tags().get_value_by_key("highway");
    std::string_view highway = highway_c ? std::string_view(highway_c) : std::string_view();

    int kph = -1;
    if (const char* ms = w.tags().get_value_by_key("maxspeed")) {
        std::string_view s(ms);

        // very simple mph support: "30 mph"
        const bool mph = (s.find("mph") != std::string_view::npos);

        const int n = parse_leading_int(s);
        if (n > 0) {
            if (mph) {
                // mph -> km/h
                kph = static_cast<int>(n * 1.60934);
            }
            else {
                kph = n;
            }
        }
    }
    if (kph <= 0) kph = default_speed_kph(highway);

    return static_cast<float>(static_cast<double>(kph) / 3.6); // m/s
}

// -------- temp records --------
#pragma pack(push, 1)
struct EdgeOSM {
    std::int64_t from{};
    std::int64_t to{};
    float speed_mps{};
};
struct EdgeDense {
    uint32_t from{};
    uint32_t to{};
    float speed_mps{};
};
#pragma pack(pop)

static_assert(sizeof(EdgeOSM) == 20, "EdgeOSM layout unexpected");
static_assert(sizeof(EdgeDense) == 12, "EdgeDense layout unexpected");

static bool read_record(std::ifstream& in, void* data, size_t bytes) {
    in.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(bytes));
    return static_cast<bool>(in);
}

// -------- external sort unique for int64 --------
static bool external_sort_unique_int64(const fs::path& input,
    const fs::path& output,
    const fs::path& runs_dir,
    size_t mem_bytes,
    std::string& err) {
    fs::create_directories(runs_dir);
    if (mem_bytes < 8 * 1024) mem_bytes = 8 * 1024;

    const size_t max_items = mem_bytes / sizeof(std::int64_t);
    std::vector<fs::path> runs;

    // chunk -> sort -> unique -> run
    {
        std::ifstream in(input, std::ios::binary);
        if (!in) { err = "Can't open nodes_raw for sort: " + input.string(); return false; }

        std::vector<std::int64_t> buf;
        buf.reserve(max_items);

        size_t run_id = 0;
        while (true) {
            buf.clear();
            std::int64_t x{};
            for (size_t i = 0; i < max_items; ++i) {
                if (!read_record(in, &x, sizeof(x))) break;
                buf.push_back(x);
            }
            if (buf.empty()) break;

            std::sort(buf.begin(), buf.end());
            buf.erase(std::unique(buf.begin(), buf.end()), buf.end());

            fs::path run_path = runs_dir / ("run_" + std::to_string(run_id++) + ".bin");
            std::ofstream out(run_path, std::ios::binary);
            if (!out) { err = "Can't create run file: " + run_path.string(); return false; }
            out.write(reinterpret_cast<const char*>(buf.data()),
                static_cast<std::streamsize>(buf.size() * sizeof(std::int64_t)));
            if (!out) { err = "Write failed for run file: " + run_path.string(); return false; }

            runs.push_back(run_path);
            if (!in) break;
        }
    }

    if (runs.empty()) {
        std::ofstream out(output, std::ios::binary);
        return static_cast<bool>(out);
    }

    if (runs.size() == 1) {
        std::error_code ec;
        fs::rename(runs[0], output, ec);
        if (ec) {
            std::ifstream src(runs[0], std::ios::binary);
            std::ofstream dst(output, std::ios::binary);
            dst << src.rdbuf();
            fs::remove(runs[0], ec);
        }
        return true;
    }

    // k-way merge unique
    struct Item { std::int64_t v; size_t i; };
    struct Cmp { bool operator()(const Item& a, const Item& b) const { return a.v > b.v; } };
    std::priority_queue<Item, std::vector<Item>, Cmp> pq;

    std::vector<std::ifstream> ins;
    ins.reserve(runs.size());
    for (auto& rp : runs) {
        ins.emplace_back(rp, std::ios::binary);
        if (!ins.back()) { err = "Can't open run file: " + rp.string(); return false; }
    }
    for (size_t i = 0; i < ins.size(); ++i) {
        std::int64_t v{};
        if (read_record(ins[i], &v, sizeof(v))) pq.push({ v, i });
    }

    std::ofstream out(output, std::ios::binary);
    if (!out) { err = "Can't open nodes_sorted_unique: " + output.string(); return false; }

    bool have_last = false;
    std::int64_t last = 0;

    while (!pq.empty()) {
        auto it = pq.top(); pq.pop();

        if (!have_last || it.v != last) {
            out.write(reinterpret_cast<const char*>(&it.v), sizeof(it.v));
            last = it.v;
            have_last = true;
        }

        std::int64_t next{};
        if (read_record(ins[it.i], &next, sizeof(next))) pq.push({ next, it.i });
    }

    std::error_code ec;
    for (auto& rp : runs) fs::remove(rp, ec);

    return true;
}

// -------- sort edges by (from,to) and keep MAX speed --------
static bool external_sort_edges_keep_fastest(const fs::path& input,
    const fs::path& output,
    const fs::path& runs_dir,
    size_t mem_bytes,
    std::string& err) {
    fs::create_directories(runs_dir);
    if (mem_bytes < sizeof(EdgeDense) * 1024) mem_bytes = sizeof(EdgeDense) * 1024;

    const size_t max_items = mem_bytes / sizeof(EdgeDense);
    std::vector<fs::path> runs;

    auto less = [](const EdgeDense& a, const EdgeDense& b) {
        if (a.from != b.from) return a.from < b.from;
        return a.to < b.to;
        };

    // chunk -> sort -> compress -> run
    {
        std::ifstream in(input, std::ios::binary);
        if (!in) { err = "Can't open edges_dense for sort: " + input.string(); return false; }

        std::vector<EdgeDense> buf;
        buf.reserve(max_items);

        size_t run_id = 0;
        while (true) {
            buf.clear();
            EdgeDense e{};
            for (size_t i = 0; i < max_items; ++i) {
                if (!read_record(in, &e, sizeof(e))) break;
                buf.push_back(e);
            }
            if (buf.empty()) break;

            std::sort(buf.begin(), buf.end(), [&](const EdgeDense& a, const EdgeDense& b) {
                if (a.from != b.from) return a.from < b.from;
                if (a.to != b.to) return a.to < b.to;
                return a.speed_mps > b.speed_mps; // fast first
                });

            std::vector<EdgeDense> outbuf;
            outbuf.reserve(buf.size());

            for (size_t i = 0; i < buf.size(); ) {
                EdgeDense best = buf[i];
                size_t j = i + 1;
                while (j < buf.size() && buf[j].from == best.from && buf[j].to == best.to) {
                    // best already has max speed due to sort order
                    ++j;
                }
                outbuf.push_back(best);
                i = j;
            }

            fs::path run_path = runs_dir / ("run_" + std::to_string(run_id++) + ".bin");
            std::ofstream out(run_path, std::ios::binary);
            if (!out) { err = "Can't create edge run: " + run_path.string(); return false; }

            out.write(reinterpret_cast<const char*>(outbuf.data()),
                static_cast<std::streamsize>(outbuf.size() * sizeof(EdgeDense)));
            if (!out) { err = "Write failed for edge run: " + run_path.string(); return false; }

            runs.push_back(run_path);
            if (!in) break;
        }
    }

    if (runs.empty()) {
        std::ofstream out(output, std::ios::binary);
        return static_cast<bool>(out);
    }

    if (runs.size() == 1) {
        std::error_code ec;
        fs::rename(runs[0], output, ec);
        if (ec) {
            std::ifstream src(runs[0], std::ios::binary);
            std::ofstream dst(output, std::ios::binary);
            dst << src.rdbuf();
            fs::remove(runs[0], ec);
        }
        return true;
    }

    // k-way merge by (from,to), keep max speed
    struct Item { EdgeDense e; size_t i; };
    struct Cmp {
        bool operator()(const Item& a, const Item& b) const {
            // min-heap by key (from,to)
            if (a.e.from != b.e.from) return a.e.from > b.e.from;
            return a.e.to > b.e.to;
        }
    };

    std::priority_queue<Item, std::vector<Item>, Cmp> pq;

    std::vector<std::ifstream> ins;
    ins.reserve(runs.size());
    for (auto& rp : runs) {
        ins.emplace_back(rp, std::ios::binary);
        if (!ins.back()) { err = "Can't open edge run: " + rp.string(); return false; }
    }

    for (size_t i = 0; i < ins.size(); ++i) {
        EdgeDense e{};
        if (read_record(ins[i], &e, sizeof(e))) pq.push({ e, i });
    }

    std::ofstream out(output, std::ios::binary);
    if (!out) { err = "Can't open edges_sorted_unique: " + output.string(); return false; }

    while (!pq.empty()) {
        Item it = pq.top(); pq.pop();

        const uint32_t key_from = it.e.from;
        const uint32_t key_to = it.e.to;

        float best_speed = it.e.speed_mps;
        std::vector<size_t> used_runs;
        used_runs.push_back(it.i);

        while (!pq.empty() && pq.top().e.from == key_from && pq.top().e.to == key_to) {
            Item same = pq.top(); pq.pop();
            if (same.e.speed_mps > best_speed) best_speed = same.e.speed_mps;
            used_runs.push_back(same.i);
        }

        EdgeDense best{ key_from, key_to, best_speed };
        out.write(reinterpret_cast<const char*>(&best), sizeof(best));
        if (!out) { err = "Write failed during edge merge."; return false; }

        // push next from each used run
        for (size_t ri : used_runs) {
            EdgeDense nxt{};
            if (read_record(ins[ri], &nxt, sizeof(nxt))) pq.push({ nxt, ri });
        }
    }

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

// -------- pass 1: ways -> nodes_raw.bin + edges_osm.bin --------
static bool pass_ways_collect(const std::string& pbf_path,
    const fs::path& tmp_nodes_raw,
    const fs::path& tmp_edges_osm,
    std::string& err) {
    std::ofstream nodes_out(tmp_nodes_raw, std::ios::binary);
    if (!nodes_out) { err = "Can't create nodes_raw: " + tmp_nodes_raw.string(); return false; }

    std::ofstream edges_out(tmp_edges_osm, std::ios::binary);
    if (!edges_out) { err = "Can't create edges_osm: " + tmp_edges_osm.string(); return false; }

    struct Handler : public osmium::handler::Handler {
        std::ofstream& nodes_out;
        std::ofstream& edges_out;

        Handler(std::ofstream& n, std::ofstream& e) : nodes_out(n), edges_out(e) {}

        void way(const osmium::Way& w) {
            const char* hw = w.tags().get_value_by_key("highway");
            if (!is_drivable_highway(hw)) return;

            const float sp = speed_mps_for_way(w);
            const int dir = oneway_dir(w.tags());
            const auto& nr = w.nodes();
            if (nr.size() < 2) return;

            // nodes_raw: dump all refs (with repeats)
            for (const auto& nref : nr) {
                const std::int64_t id = static_cast<std::int64_t>(nref.ref());
                nodes_out.write(reinterpret_cast<const char*>(&id), sizeof(id));
            }

            // edges_osm: by segments
            for (size_t i = 1; i < nr.size(); ++i) {
                const std::int64_t a = static_cast<std::int64_t>(nr[i - 1].ref());
                const std::int64_t b = static_cast<std::int64_t>(nr[i].ref());

                if (dir == 1) {
                    EdgeOSM e{ a, b, sp };
                    edges_out.write(reinterpret_cast<const char*>(&e), sizeof(e));
                }
                else if (dir == -1) {
                    EdgeOSM e{ b, a, sp };
                    edges_out.write(reinterpret_cast<const char*>(&e), sizeof(e));
                }
                else {
                    EdgeOSM e1{ a, b, sp };
                    EdgeOSM e2{ b, a, sp };
                    edges_out.write(reinterpret_cast<const char*>(&e1), sizeof(e1));
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
        err = "Write failed during WAYS pass.";
        return false;
    }
    return true;
}

// -------- pass 2: nodes -> coords only for required osm_ids --------
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

            const std::int64_t id = static_cast<std::int64_t>(n.id());
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

// -------- edges osm -> dense(with speed) --------
static bool convert_edges_to_dense(const fs::path& tmp_edges_osm,
    const std::vector<std::int64_t>& osm_ids,
    const fs::path& tmp_edges_dense,
    std::string& err) {
    std::ifstream in(tmp_edges_osm, std::ios::binary);
    if (!in) { err = "Can't open edges_osm: " + tmp_edges_osm.string(); return false; }

    std::ofstream out(tmp_edges_dense, std::ios::binary);
    if (!out) { err = "Can't create edges_dense: " + tmp_edges_dense.string(); return false; }

    EdgeOSM e{};
    while (read_record(in, &e, sizeof(e))) {
        uint32_t a = 0, b = 0;
        if (!map_osm_to_dense(osm_ids, e.from, a)) continue;
        if (!map_osm_to_dense(osm_ids, e.to, b)) continue;

        EdgeDense d{ a, b, e.speed_mps };
        out.write(reinterpret_cast<const char*>(&d), sizeof(d));
    }

    if (!out) { err = "Write failed for edges_dense."; return false; }
    return true;
}

// -------- write graph.bin v2 light (speed only) --------
static bool write_graph_bin_v2(const fs::path& edges_sorted_unique,
    const std::vector<std::int64_t>& osm_ids,
    const std::vector<double>& lat,
    const std::vector<double>& lon,
    const std::string& out_bin_path,
    std::string& err) {
    const uint64_t N64 = static_cast<uint64_t>(osm_ids.size());
    if (N64 == 0) { err = "0 nodes after filtering."; return false; }
    if (N64 > std::numeric_limits<uint32_t>::max()) { err = "N too large for uint32 indices."; return false; }
    const uint32_t N = static_cast<uint32_t>(N64);

    // degree + M
    std::vector<uint32_t> degree(N, 0);
    uint32_t M = 0;
    {
        std::ifstream in(edges_sorted_unique, std::ios::binary);
        if (!in) { err = "Can't open edges_sorted_unique: " + edges_sorted_unique.string(); return false; }
        EdgeDense e{};
        while (read_record(in, &e, sizeof(e))) {
            if (e.from < N) degree[e.from]++;
            M++;
        }
    }

    // offsets (cumulative end)
    std::vector<uint32_t> offsets(N, 0);
    uint64_t total = 0;
    for (uint32_t i = 0; i < N; ++i) {
        total += degree[i];
        if (total > std::numeric_limits<uint32_t>::max()) {
            err = "Too many edges for uint32 offsets.";
            return false;
        }
        offsets[i] = static_cast<uint32_t>(total);
    }

    std::ofstream out(out_bin_path, std::ios::binary);
    if (!out) { err = "Can't open output bin: " + out_bin_path; return false; }

    // header
    const char magic[4] = { 'G','B','I','N' };
    const uint32_t version = 2;
    const uint32_t flags = 1; // speed_present

    out.write(magic, 4);
    out.write(reinterpret_cast<const char*>(&version), sizeof(uint32_t));
    out.write(reinterpret_cast<const char*>(&N64), sizeof(uint64_t));
    out.write(reinterpret_cast<const char*>(&flags), sizeof(uint32_t));

    // ids 0..N-1
    for (uint32_t i = 0; i < N; ++i) {
        out.write(reinterpret_cast<const char*>(&i), sizeof(uint32_t));
    }

    // coords double
    for (uint32_t i = 0; i < N; ++i) {
        out.write(reinterpret_cast<const char*>(&lat[i]), sizeof(double));
        out.write(reinterpret_cast<const char*>(&lon[i]), sizeof(double));
    }

    // offsets
    out.write(reinterpret_cast<const char*>(offsets.data()),
        static_cast<std::streamsize>(N * sizeof(uint32_t)));

    // adjacency (pass 1)
    {
        std::ifstream in(edges_sorted_unique, std::ios::binary);
        if (!in) { err = "Can't reopen edges_sorted_unique."; return false; }
        EdgeDense e{};
        while (read_record(in, &e, sizeof(e))) {
            out.write(reinterpret_cast<const char*>(&e.to), sizeof(uint32_t));
        }
    }

    // speed_mps array (pass 2), same order as adjacency
    {
        std::ifstream in(edges_sorted_unique, std::ios::binary);
        if (!in) { err = "Can't reopen edges_sorted_unique for speed."; return false; }
        EdgeDense e{};
        while (read_record(in, &e, sizeof(e))) {
            out.write(reinterpret_cast<const char*>(&e.speed_mps), sizeof(float));
        }
    }

    if (!out) { err = "Write failed for output bin."; return false; }
    return true;
}

bool preprocess_pbf_to_bin(const std::string& pbf_path,
    const std::string& out_bin_path,
    const std::string& out_map_csv_path,
    std::string& err) {
    try {
        // temp dir next to output
        fs::path outp = fs::absolute(out_bin_path);
        fs::path tmp_dir = outp.parent_path() / (outp.filename().string() + ".tmp");
        fs::create_directories(tmp_dir);

        const fs::path nodes_raw = tmp_dir / "nodes_raw.bin";
        const fs::path edges_osm = tmp_dir / "edges_osm.bin";
        const fs::path nodes_sorted = tmp_dir / "nodes_sorted_unique.bin";
        const fs::path edges_dense = tmp_dir / "edges_dense.bin";
        const fs::path edges_sorted = tmp_dir / "edges_dense_sorted_unique.bin";

        const size_t MEM_BYTES = 256ull * 1024ull * 1024ull;

        // 1) ways pass
        if (!pass_ways_collect(pbf_path, nodes_raw, edges_osm, err)) return false;

        // 2) sort unique nodes
        if (!external_sort_unique_int64(nodes_raw, nodes_sorted, tmp_dir / "runs_nodes", MEM_BYTES, err)) return false;

        // load osm_ids
        std::vector<std::int64_t> osm_ids;
        {
            std::ifstream in(nodes_sorted, std::ios::binary);
            if (!in) { err = "Can't open nodes_sorted_unique."; return false; }
            std::int64_t id{};
            while (read_record(in, &id, sizeof(id))) osm_ids.push_back(id);
        }
        if (osm_ids.empty()) { err = "0 nodes after filtering ways."; return false; }

        // 3) nodes pass -> coords
        std::vector<double> lat, lon;
        if (!pass_nodes_fill_coords(pbf_path, osm_ids, lat, lon, err)) return false;

        // 4) edges osm -> dense (with speed)
        if (!convert_edges_to_dense(edges_osm, osm_ids, edges_dense, err)) return false;

        // 5) sort edges unique (from,to), keep max speed
        if (!external_sort_edges_keep_fastest(edges_dense, edges_sorted, tmp_dir / "runs_edges", MEM_BYTES, err)) return false;

        // 6) write graph.bin v2 light (speed only)
        if (!write_graph_bin_v2(edges_sorted, osm_ids, lat, lon, out_bin_path, err)) return false;

        // mapping csv
        if (!out_map_csv_path.empty()) {
            std::ofstream csv(out_map_csv_path);
            if (!csv) { err = "Can't open map csv: " + out_map_csv_path; return false; }
            csv << "dense_id,osm_id,lat,lon\n";
            for (uint32_t i = 0; i < static_cast<uint32_t>(osm_ids.size()); ++i) {
                csv << (i + 1) << "," << osm_ids[i] << "," << lat[i] << "," << lon[i] << "\n";
            }
        }

        std::error_code ec;
        fs::remove_all(tmp_dir, ec);
        return true;
    }
    catch (const std::exception& e) {
        err = std::string("Preprocess exception: ") + e.what();
        return false;
    }
}