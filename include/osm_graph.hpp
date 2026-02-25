#pragma once
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <string>

struct OsmNode {
    std::int64_t osm_id{};
    double lat{};
    double lon{};
};

struct OsmEdge {
    std::int64_t from_osm{};
    std::int64_t to_osm{};
    double length_m{};
    bool oneway{};
};

struct OsmGraph {
    std::unordered_map<std::int64_t, OsmNode> nodes;
    std::vector<OsmEdge> edges;
};

OsmGraph build_graph_from_pbf(const std::string& pbf_path);
