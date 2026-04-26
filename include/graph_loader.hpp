#pragma once

#include "route.hpp"

#include <cstdint>
#include <string>
#include <vector>

struct NodeCoord {
    double lat{};
    double lon{};
};

struct LoadedGraph {
    std::uint32_t N = 0;
    std::vector<NodeCoord> coords;
    std::vector<std::uint32_t> offsets;
    std::vector<std::uint32_t> adjacency;
    std::vector<float> speeds;
    bool speed_present = false;

    std::uint32_t find_nearest(double lat, double lon) const;
};

LoadedGraph load_graph_bin(const std::string& bin_path);
void load_routes_from_file(const std::string& result_txt, std::vector<Route>& routes_out);
