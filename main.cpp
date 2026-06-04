#include "visualizer.hpp"
#include "osm_graph.hpp"
#include "preprocess.hpp"
#include "search.h"
#include "geocode.hpp"
#include "tsp.hpp"
#include "csv_reader.hpp"
#include "vrp_solver.hpp" 

#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <system_error>
#include <vector>

namespace fs = std::filesystem;

static void usage(const char* exe) {
    std::cerr
        << "Usage:\n"
        << "  " << exe << " osm-info   --pbf <file.osm.pbf>\n"
        << "  " << exe << " preprocess --pbf <file.osm.pbf> --out <graph.bin> [--map <id_map.csv>]\n"
        << "  " << exe << " search     --graph <graph.bin> --in <input.txt> [--queries <queries.txt|dir>] --out <result.txt> --metric <time/distance> [--full]\n"
        << "  " << exe << " visualize  --graph <graph.bin> [--result <result.txt>]\n"
        << "  " << exe << " tsp        --graph <graph.bin> --in <input.txt> --out <result.txt> --map <id_map.csv> --metric <time/distance> [--full]\n"
        << "\n"
        << "Notes:\n"
        << "  - preprocess creates a dense graph: query node ids are 1..N\n"
        << "  - id_map.csv contains mapping dense_id -> original osm_id and coordinates\n"
        << "  - search reads coordinate input.txt, writes generated queries.txt, then routes using that exact queries.txt\n"
        << "  - tsp input format: one line with space-separated lat lon lat lon ... (first point is base)\n";
}

static std::string take_value(int& i, int argc, char** argv) {
    if (i + 1 >= argc) return {};
    return argv[++i];
}

struct ResolveResult {
    fs::path path;
    bool found = false;
    std::vector<fs::path> tried;
};

static ResolveResult resolve_existing(const fs::path& p,
    const fs::path& cwd,
    const fs::path& exe_dir) {
    ResolveResult rr;
    if (p.empty()) {
        rr.path = p;
        return rr;
    }

    std::error_code ec;
    std::vector<fs::path> tries;

    if (p.is_absolute()) {
        tries.push_back(p);
    }
    else {
        tries.push_back(p);
        tries.push_back(cwd / p);
        tries.push_back(exe_dir / p);
        tries.push_back(exe_dir.parent_path() / p);
        tries.push_back(exe_dir.parent_path().parent_path() / p);

        fs::path abs = fs::absolute(p, ec);
        if (!ec && !abs.empty()) tries.push_back(abs);
    }

    rr.tried = tries;
    for (const auto& t : tries) {
        if (fs::exists(t, ec) && !ec) {
            rr.path = t;
            rr.found = true;
            return rr;
        }
        ec.clear();
    }

    rr.path = p;
    return rr;
}

static ResolveResult resolve_input_txt(const fs::path& arg,
    const fs::path& cwd,
    const fs::path& exe_dir) {
    ResolveResult rr = resolve_existing(arg, cwd, exe_dir);

    std::error_code ec;
    if (rr.found && fs::is_directory(rr.path, ec) && !ec) {
        ResolveResult as_dir = resolve_existing(rr.path / "input.txt", cwd, exe_dir);
        as_dir.tried.insert(as_dir.tried.begin(), rr.tried.begin(), rr.tried.end());
        return as_dir;
    }

    if (rr.found) return rr;

    // Also support passing a directory that is not found by the generic resolver first.
    // This is only a fallback; if --in is a file, the file path is used directly.
    ResolveResult dir_rr;
    std::vector<fs::path> tries;
    if (arg.is_absolute()) {
        tries.push_back(arg / "input.txt");
    }
    else {
        tries.push_back(cwd / arg / "input.txt");
        tries.push_back(exe_dir / arg / "input.txt");
        tries.push_back(exe_dir.parent_path() / arg / "input.txt");
        tries.push_back(exe_dir.parent_path().parent_path() / arg / "input.txt");
    }

    dir_rr.tried = rr.tried;
    dir_rr.tried.insert(dir_rr.tried.end(), tries.begin(), tries.end());

    for (const auto& t : tries) {
        if (fs::exists(t, ec) && !ec) {
            dir_rr.path = t;
            dir_rr.found = true;
            return dir_rr;
        }
        ec.clear();
    }

    dir_rr.path = arg;
    return dir_rr;
}

static fs::path make_output_path(const fs::path& p, const fs::path& cwd) {
    if (p.empty()) return p;
    if (p.is_absolute()) return p;
    return cwd / p;
}

static fs::path make_queries_output_path(const std::string& queries_arg,
    const fs::path& coordinate_input_path,
    const fs::path& cwd) {
    if (queries_arg.empty()) {
        return coordinate_input_path.parent_path() / "queries.txt";
    }

    fs::path q = make_output_path(fs::path(queries_arg), cwd);

    std::error_code ec;
    if (fs::exists(q, ec) && !ec && fs::is_directory(q, ec) && !ec) {
        return q / "queries.txt";
    }

    if (q.filename().empty()) {
        return q / "queries.txt";
    }

    return q;
}

static void print_not_found(const char* label, const ResolveResult& rr) {
    std::cerr << label << " not found.\n";
    std::cerr << "Tried:\n";
    for (const auto& t : rr.tried) {
        std::cerr << "  - " << t.string() << "\n";
    }
}

static bool ensure_parent_directory(const fs::path& file_path) {
    const fs::path parent = file_path.parent_path();
    if (parent.empty()) return true;

    std::error_code ec;
    fs::create_directories(parent, ec);
    return !ec;
}

static fs::path find_id_map_csv(const fs::path& graph_path,
    const fs::path& coordinate_input_path,
    const fs::path& queries_path,
    const fs::path& cwd,
    const fs::path& exe_dir,
    std::vector<fs::path>& candidates) {
    candidates = {
        graph_path.parent_path() / "id_map.csv",
        coordinate_input_path.parent_path() / "id_map.csv",
        queries_path.parent_path() / "id_map.csv",
        cwd / "id_map.csv",
        exe_dir / "id_map.csv",
        exe_dir.parent_path() / "id_map.csv",
        exe_dir.parent_path().parent_path() / "id_map.csv"
    };

    std::error_code ec;
    for (const auto& candidate : candidates) {
        if (!candidate.empty() && fs::exists(candidate, ec) && !ec) {
            return candidate;
        }
        ec.clear();
    }
    return {};
}

int main(int argc, char** argv) {
    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    const std::string cmd = argv[1];

    std::error_code ec;
    const fs::path cwd = fs::current_path(ec);
    ec.clear();
    const fs::path exe_dir = fs::absolute(argv[0], ec).parent_path();

    // ---------------- osm-info ----------------
    if (cmd == "osm-info") {
        std::string pbf;

        for (int i = 2; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "--pbf") pbf = take_value(i, argc, argv);
            else if (a == "--help" || a == "-h") { usage(argv[0]); return 0; }
        }

        if (pbf.empty()) {
            usage(argv[0]);
            return 1;
        }

        auto rr = resolve_existing(pbf, cwd, exe_dir);
        std::cerr << "cwd: " << cwd.string() << "\n";
        std::cerr << "exe dir: " << exe_dir.string() << "\n";

        if (!rr.found) {
            print_not_found("PBF", rr);
            return 1;
        }

        std::cerr << "pbf: " << rr.path.string() << "\n";

        try {
            OsmGraph g = build_graph_from_pbf(rr.path.string());
            std::cout << "OK\n";
            std::cout << "Nodes: " << g.nodes.size() << "\n";
            std::cout << "Edges: " << g.edges.size() << "\n";
            return 0;
        }
        catch (const std::system_error& e) {
            std::cerr << "system_error: " << e.what()
                << "\ncode: " << e.code()
                << "\nvalue: " << e.code().value()
                << "\nmessage: " << e.code().message()
                << "\n";
            return 1;
        }
        catch (const std::exception& e) {
            std::cerr << "exception: " << e.what() << "\n";
            return 1;
        }
    }

    // ---------------- preprocess ----------------
    if (cmd == "preprocess") {
        std::string pbf, out_bin, out_map = "id_map.csv";

        for (int i = 2; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "--pbf") pbf = take_value(i, argc, argv);
            else if (a == "--out") out_bin = take_value(i, argc, argv);
            else if (a == "--map") out_map = take_value(i, argc, argv);
            else if (a == "--help" || a == "-h") { usage(argv[0]); return 0; }
        }

        if (pbf.empty() || out_bin.empty()) {
            usage(argv[0]);
            return 1;
        }

        auto pbf_rr = resolve_existing(pbf, cwd, exe_dir);
        if (!pbf_rr.found) {
            std::cerr << "cwd: " << cwd.string() << "\n";
            print_not_found("PBF", pbf_rr);
            return 1;
        }

        const fs::path out_bin_path = make_output_path(out_bin, cwd);
        const fs::path out_map_path = out_map.empty() ? fs::path{} : make_output_path(out_map, cwd);

        if (!ensure_parent_directory(out_bin_path)) {
            std::cerr << "Cannot create output directory for: " << out_bin_path.string() << "\n";
            return 1;
        }
        if (!out_map_path.empty() && !ensure_parent_directory(out_map_path)) {
            std::cerr << "Cannot create map output directory for: " << out_map_path.string() << "\n";
            return 1;
        }

        std::cerr << "cwd: " << cwd.string() << "\n";
        std::cerr << "pbf: " << pbf_rr.path.string() << "\n";
        std::cerr << "out: " << out_bin_path.string() << "\n";
        if (!out_map_path.empty()) std::cerr << "map: " << out_map_path.string() << "\n";

        std::string err;
        if (!preprocess_pbf_to_bin(pbf_rr.path.string(),
            out_bin_path.string(),
            out_map_path.empty() ? std::string{} : out_map_path.string(),
            err)) {
            std::cerr << "Preprocess failed: " << err << "\n";
            return 1;
        }

        std::cout << "Preprocess OK\n";
        return 0;
    }

    // ---------------- search ----------------
    if (cmd == "search") {
        std::string graph, input_txt, queries_arg, out;
        bool full = false;
        SearchMetric metric = SearchMetric::Distance;

        for (int i = 2; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "--graph") graph = take_value(i, argc, argv);
            else if (a == "--in") input_txt = take_value(i, argc, argv);
            else if (a == "--queries") queries_arg = take_value(i, argc, argv);
            else if (a == "--out") out = take_value(i, argc, argv);
            else if (a == "--full") full = true;
            else if (a == "--help" || a == "-h") { usage(argv[0]); return 0; }
            else if (a == "--metric") {
                const std::string m = take_value(i, argc, argv);
                if (m == "distance") metric = SearchMetric::Distance;
                else if (m == "time") metric = SearchMetric::Time;
                else { std::cerr << "Unknown metric: " << m << "\n"; return 1; }
            }
        }

        if (graph.empty() || input_txt.empty() || out.empty()) {
            usage(argv[0]);
            return 1;
        }

        auto graph_rr = resolve_existing(graph, cwd, exe_dir);
        if (!graph_rr.found) {
            std::cerr << "cwd: " << cwd.string() << "\n";
            print_not_found("Graph", graph_rr);
            return 1;
        }

        // Important: --in is resolved as the actual coordinate input file.
        // There is no "parent_path()/input.txt" substitution here.
        auto input_rr = resolve_input_txt(input_txt, cwd, exe_dir);
        if (!input_rr.found) {
            std::cerr << "cwd: " << cwd.string() << "\n";
            print_not_found("Input", input_rr);
            std::cerr << "\nExpected coordinate input format:\n";
            std::cerr << "  lat1 lon1 lat2 lon2\n";
            std::cerr << "Example:\n";
            std::cerr << "  57.2834 34.8924 57.3558 34.8520\n";
            return 1;
        }

        const fs::path queries_path = make_queries_output_path(queries_arg, input_rr.path, cwd);
        const fs::path out_path = make_output_path(out, cwd);

        if (!ensure_parent_directory(queries_path)) {
            std::cerr << "Cannot create queries output directory for: " << queries_path.string() << "\n";
            return 1;
        }
        if (!ensure_parent_directory(out_path)) {
            std::cerr << "Cannot create result output directory for: " << out_path.string() << "\n";
            return 1;
        }

        std::vector<fs::path> map_candidates;
        const fs::path map_csv_path = find_id_map_csv(graph_rr.path,
            input_rr.path,
            queries_path,
            cwd,
            exe_dir,
            map_candidates);
        if (map_csv_path.empty()) {
            std::cerr << "\nERROR: id_map.csv not found\n";
            std::cerr << "Expected locations:\n";
            for (const auto& candidate : map_candidates) {
                std::cerr << "  - " << candidate.string() << "\n";
            }
            return 1;
        }

        std::cerr << "cwd:     " << cwd.string() << "\n";
        std::cerr << "graph:   " << graph_rr.path.string() << "\n";
        std::cerr << "input:   " << input_rr.path.string() << "\n";
        std::cerr << "map:     " << map_csv_path.string() << "\n";
        std::cerr << "queries: " << queries_path.string() << "\n";
        std::cerr << "out:     " << out_path.string() << "\n";

        std::string err;
        std::cerr << "Converting coordinates from input.txt to dense node-id queries...\n";
        if (!operations_research::convert_coordinates_to_ids(map_csv_path.string(),
            input_rr.path.string(),
            queries_path.string(),
            err)) {
            std::cerr << "\nERROR: Conversion failed: " << err << "\n";
            return 1;
        }

        std::error_code exists_ec;
        if (!fs::exists(queries_path, exists_ec) || exists_ec) {
            std::cerr << "\nERROR: generated queries.txt not found at: " << queries_path.string() << "\n";
            return 1;
        }

        FILE* gf = std::fopen(graph_rr.path.string().c_str(), "rb");
        if (!gf) {
            std::cerr << "Can't open graph: " << graph_rr.path.string() << "\n";
            return 1;
        }

        // Important: run_search reads exactly the generated queries_path.
        FILE* inf = std::fopen(queries_path.string().c_str(), "r");
        if (!inf) {
            std::cerr << "Can't open generated queries: " << queries_path.string() << "\n";
            std::fclose(gf);
            return 1;
        }

        FILE* outf = std::fopen(out_path.string().c_str(), "w");
        if (!outf) {
            std::cerr << "Can't open output: " << out_path.string() << "\n";
            std::fclose(gf);
            std::fclose(inf);
            return 1;
        }

        run_search(gf, inf, outf, full, metric);

        std::fclose(outf);
        std::fclose(inf);
        std::fclose(gf);

        std::cout << "Search OK\n";
        return 0;
    }
    // ---------------- tsp ----------------
    if (cmd == "tsp") {
        std::string graph, in_txt, csv_points, out, map_csv;
        bool full = false;
        SearchMetric metric = SearchMetric::Distance;

        for (int i = 2; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "--graph") graph = take_value(i, argc, argv);
            else if (a == "--in") in_txt = take_value(i, argc, argv);
            else if (a == "--csv") csv_points = take_value(i, argc, argv);
            else if (a == "--out") out = take_value(i, argc, argv);
            else if (a == "--map") map_csv = take_value(i, argc, argv);
            else if (a == "--full") full = true;
            else if (a == "--metric") {
                const std::string m = take_value(i, argc, argv);
                if (m == "distance") metric = SearchMetric::Distance;
                else if (m == "time") metric = SearchMetric::Time;
                else { std::cerr << "Unknown metric: " << m << "\n"; return 1; }
            }
            else if (a == "--help" || a == "-h") { usage(argv[0]); return 0; }
        }

        // 1.                                                                  
        if (!in_txt.empty() && !csv_points.empty()) {
            std::cerr << "Error: Flags --in and --csv are mutually exclusive. Choose only one source.\n";
            return 1;
        }
        if (in_txt.empty() && csv_points.empty()) {
            std::cerr << "Error: Missing source points. You must specify either --in <input.txt> or --csv <points.csv>.\n";
            return 1;
        }

        // 2.                                           
        if (graph.empty() || out.empty() || map_csv.empty()) {
            std::cerr << "Usage:\n"
                << "  TXT mode: " << argv[0] << " tsp --graph <graph.bin> --in <input.txt> --out <result.txt> --map <id_map.csv> [--full] [--metric time|distance]\n"
                << "  CSV mode: " << argv[0] << " tsp --graph <graph.bin> --csv <points.csv> --out <result.txt> --map <id_map.csv> [--full] [--metric time|distance]\n";
            return 1;
        }

        // 3.                       (                        )
        auto graph_rr = resolve_existing(graph, cwd, exe_dir);
        if (!graph_rr.found) {
            print_not_found("Graph", graph_rr);
            return 1;
        }

        auto map_rr = resolve_existing(map_csv, cwd, exe_dir);
        if (!map_rr.found) {
            print_not_found("Map CSV", map_rr);
            return 1;
        }

        const fs::path out_path = make_output_path(out, cwd);
        std::string err;
        std::string target_input_file;
        bool is_temporary_file = false;

        // 4.                                                             
        if (!csv_points.empty()) {
            //                CSV (       tsp-csv)
            auto csv_rr = resolve_existing(csv_points, cwd, exe_dir);
            if (!csv_rr.found) {
                print_not_found("CSV", csv_rr);
                return 1;
            }

            std::vector<TspPoint> points;
            if (!load_tsp_points_from_csv(csv_rr.path.string(), points, err)) {
                std::cerr << "Failed to load CSV: " << err << "\n";
                return 1;
            }

            //                                ,         solve_tsp                     .txt
            target_input_file = "temp_tsp_input.txt";
            write_tsp_input_file(target_input_file, points);
            is_temporary_file = true;
        }
        else {
            //                                                 
            auto in_rr = resolve_existing(in_txt, cwd, exe_dir);
            if (!in_rr.found) {
                print_not_found("Input TXT", in_rr);
                return 1;
            }
            target_input_file = in_rr.path.string();
        }

        // 5.                                     TSP
        std::cout << "Starting TSP Optimization...\n";
        bool success = solve_tsp(target_input_file,
            graph_rr.path.string(),
            out_path.string(),
            map_rr.path.string(),
            full,
            metric,
            err);

        //                                     CSV                         
        if (is_temporary_file) {
            std::remove(target_input_file.c_str());
        }

        if (!success) {
            std::cerr << "TSP failed: " << err << "\n";
            return 1;
        }

        std::cout << "TSP OK. Results saved to " << out_path.string() << "\n";
        return 0;
    }
    // ---------------- vrp ----------------
    if (cmd == "vrp") {
        std::string graph, csv, fleet, out, map_csv;
        SearchMetric metric = SearchMetric::Distance;
        bool full_output = false; //                                        

        for (int i = 2; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "--graph") graph = take_value(i, argc, argv);
            else if (a == "--csv") csv = take_value(i, argc, argv);
            else if (a == "--fleet") fleet = take_value(i, argc, argv);
            else if (a == "--out") out = take_value(i, argc, argv);
            else if (a == "--map") map_csv = take_value(i, argc, argv);
            else if (a == "--full") {
                full_output = true; //                                        
            }
            else if (a == "--metric") {
                const std::string m = take_value(i, argc, argv);
                if (m == "distance") metric = SearchMetric::Distance;
                else if (m == "time") metric = SearchMetric::Time;
                else { std::cerr << "Unknown metric: " << m << "\n"; return 1; }
            }
            else if (a == "--help" || a == "-h") { usage(argv[0]); return 0; }
        }

        if (graph.empty() || csv.empty() || fleet.empty() || out.empty() || map_csv.empty()) {
            std::cerr << "Usage: " << argv[0] << " vrp --graph <graph.bin> --csv <points.csv> --fleet <fleet.csv> --out <result.txt> --map <id_map.csv> [--metric time|distance] [--full]\n";
            return 1;
        }

        auto graph_rr = resolve_existing(graph, cwd, exe_dir);
        if (!graph_rr.found) { print_not_found("Graph", graph_rr); return 1; }

        auto csv_rr = resolve_existing(csv, cwd, exe_dir);
        if (!csv_rr.found) { print_not_found("CSV", csv_rr); return 1; }

        auto fleet_rr = resolve_existing(fleet, cwd, exe_dir);
        if (!fleet_rr.found) { print_not_found("Fleet CSV", fleet_rr); return 1; }

        auto map_rr = resolve_existing(map_csv, cwd, exe_dir);
        if (!map_rr.found) { print_not_found("Map CSV", map_rr); return 1; }

        const fs::path out_path = make_output_path(out, cwd);

        std::string err;
        std::cout << "Starting VRP Optimization with Time Windows...\n";
        if (!solve_new_vrp(csv_rr.path.string(),
            fleet_rr.path.string(),
            graph_rr.path.string(),
            map_rr.path.string(),
            out_path.string(),
            metric,
            full_output, //                               
            err)) {
            std::cerr << "VRP failed: " << err << "\n";
            return 1;
        }

        std::cout << "VRP OK. Results saved to " << out_path.string() << "\n";
        return 0;
    }
    // ---------------- visualize ----------------
    if (cmd == "visualize") {
        std::string graph, result_txt;
        for (int i = 2; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "--graph") graph = take_value(i, argc, argv);
            else if (a == "--result") result_txt = take_value(i, argc, argv);
            else if (a == "--help" || a == "-h") { usage(argv[0]); return 0; }
        }

        if (graph.empty()) {
            usage(argv[0]);
            return 1;
        }

        auto graph_rr = resolve_existing(graph, cwd, exe_dir);
        if (!graph_rr.found) {
            print_not_found("Graph", graph_rr);
            return 1;
        }

        GraphVisualizer viz;
        viz.load(graph_rr.path.string());

        if (!result_txt.empty()) {
            auto res_rr = resolve_existing(result_txt, cwd, exe_dir);
            if (res_rr.found) {
                std::vector<Route> loaded;
                load_routes_from_file(res_rr.path.string(), loaded);
                for (auto& r : loaded) viz.add_route(r);
                std::cerr << "Loaded routes: " << loaded.size() << "\n";
            }
            else {
                print_not_found("Result", res_rr);
                std::cerr << "Continuing in map-only mode.\n";
            }
        }
        else {
            std::cerr << "No --result specified; drawing map only.\n";
        }

        std::cout << "Starting visualizer...\n";
        viz.run();
        return 0;
    }

    usage(argv[0]);
    return 1;
}
