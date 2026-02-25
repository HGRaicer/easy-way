#include "osm_graph.hpp"
#include "preprocess.hpp"
#include "search.h"

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <cstdio>
#include <system_error>

#ifdef _WIN32
#include <windows.h>
#endif

namespace fs = std::filesystem;

static void enable_utf8_console() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
}

static void usage(const char* exe) {
    std::cerr
        << "Usage:\n"
        << "  " << exe << " osm-info   --pbf <file.osm.pbf>\n"
        << "  " << exe << " preprocess --pbf <file.osm.pbf> --out <graph.bin> [--map <map.csv>]\n"
        << "  " << exe << " search     --graph <graph.bin> --in <queries.txt> --out <result.txt> [--full]\n"
        << "\n"
        << "Notes:\n"
        << "  - preprocess creates a dense graph: query node ids are 1..N\n"
        << "  - map.csv contains mapping dense_id -> original osm_id\n";
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
        rr.found = false;
        return rr;
    }

    std::error_code ec;

    // Если абсолютный и существует
    if (p.is_absolute() && fs::exists(p, ec) && !ec) {
        rr.path = p;
        rr.found = true;
        rr.tried = { p };
        return rr;
    }

    // Пробуем набор типичных мест
    std::vector<fs::path> tries;
    tries.push_back(p);
    tries.push_back(cwd / p);
    tries.push_back(exe_dir / p);
    tries.push_back(exe_dir.parent_path() / p);
    tries.push_back(exe_dir.parent_path().parent_path() / p);

    // И абсолютный вариант
    fs::path abs = fs::absolute(p, ec);
    if (!abs.empty()) tries.push_back(abs);

    rr.tried = tries;

    for (const auto& t : tries) {
        if (fs::exists(t, ec) && !ec) {
            rr.path = t;
            rr.found = true;
            return rr;
        }
    }

    rr.path = p;
    rr.found = false;
    return rr;
}

static fs::path make_output_path(const fs::path& p, const fs::path& cwd) {
    if (p.empty()) return p;
    if (p.is_absolute()) return p;
    return cwd / p; // удобно: всё выходит в корень проекта, если запуск из bin
}

static void print_not_found(const char* label, const ResolveResult& rr) {
    std::cerr << label << " not found.\n";
    std::cerr << "Tried:\n";
    for (const auto& t : rr.tried) {
        std::cerr << "  - " << t.string() << "\n";
    }
}

int main(int argc, char** argv) {
    enable_utf8_console();

    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    const std::string cmd = argv[1];

    std::error_code ec;
    const fs::path cwd = fs::current_path(ec);
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
        std::cerr << "exe: " << (exe_dir / "graph_builder.exe").string() << "\n";

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
        std::string graph, in, out;
        bool full = false;

        for (int i = 2; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "--graph") graph = take_value(i, argc, argv);
            else if (a == "--in") in = take_value(i, argc, argv);
            else if (a == "--out") out = take_value(i, argc, argv);
            else if (a == "--full") full = true;
            else if (a == "--help" || a == "-h") { usage(argv[0]); return 0; }
        }

        if (graph.empty() || in.empty() || out.empty()) {
            usage(argv[0]);
            return 1;
        }

        auto graph_rr = resolve_existing(graph, cwd, exe_dir);
        if (!graph_rr.found) {
            std::cerr << "cwd: " << cwd.string() << "\n";
            print_not_found("Graph", graph_rr);
            return 1;
        }

        auto in_rr = resolve_existing(in, cwd, exe_dir);
        if (!in_rr.found) {
            std::cerr << "cwd: " << cwd.string() << "\n";
            print_not_found("Input", in_rr);
            return 1;
        }

        const fs::path out_path = make_output_path(out, cwd);

        std::cerr << "cwd: " << cwd.string() << "\n";
        std::cerr << "graph: " << graph_rr.path.string() << "\n";
        std::cerr << "in:    " << in_rr.path.string() << "\n";
        std::cerr << "out:   " << out_path.string() << "\n";

        FILE* gf = fopen(graph_rr.path.string().c_str(), "rb");
        if (!gf) {
            std::cerr << "Can't open graph: " << graph_rr.path.string() << "\n";
            return 1;
        }

        FILE* inf = fopen(in_rr.path.string().c_str(), "r");
        if (!inf) {
            std::cerr << "Can't open input: " << in_rr.path.string() << "\n";
            fclose(gf);
            return 1;
        }

        FILE* outf = fopen(out_path.string().c_str(), "w");
        if (!outf) {
            std::cerr << "Can't open output: " << out_path.string() << "\n";
            fclose(gf);
            fclose(inf);
            return 1;
        }

        run_search(gf, inf, outf, full);

        fclose(outf);
        fclose(inf);
        fclose(gf);

        std::cout << "Search OK\n";
        return 0;
    }

    usage(argv[0]);
    return 1;
}
