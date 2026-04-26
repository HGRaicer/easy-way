#pragma once

#include "graph_loader.hpp"
#include "route.hpp"

#include <cstdint>
#include <string>
#include <vector>

class QtGraphWidget;

class GraphVisualizer {
public:
    void load(const std::string& bin_path);
    void add_route(const Route& r) { routes.push_back(r); }
    void add_waypoint(double lat, double lon);
    void clear_routes() { routes.clear(); }
    void clear_waypoints() { waypoints.clear(); }

    void run();

private:
    struct WorldPoint {
        double x = 0.0;
        double y = 0.0;
    };

    LoadedGraph graph;
    std::vector<Route> routes;
    std::vector<Waypoint> waypoints;

    std::vector<WorldPoint> world_coords;
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;

    std::vector<std::vector<std::uint32_t>> grid;
    int grid_cols = 0;
    int grid_rows = 0;
    double cell_size = 0.0;

    WorldPoint to_world(double lat, double lon) const;
    WorldPoint to_world(std::uint32_t node) const;
    void build_spatial_grid();

    friend class QtGraphWidget;
};
