#pragma once

#include <cstdint>
#include <string>
#include <vector>
struct RouteTarget {
    std::uint32_t dense_id; // Скорректированный 0-based ID для графа
    double arrival_time_s;  // Время прибытия в секундах (0.0 для TSP)
    bool is_depot;          // Флаг депо (для покраски в другой цвет)
};

struct Rgba {
    std::uint8_t r = 74;
    std::uint8_t g = 144;
    std::uint8_t b = 226;
    std::uint8_t a = 255;
};

struct Route {
    std::vector<std::uint32_t> nodes;     
    double total_time_s = 0.0;
    double total_dist_m = 0.0;
    std::string label = "Route";
    Rgba color{ 74, 144, 226, 255 };
    bool visible = true;

    std::vector<RouteTarget> targets;
};

struct Waypoint {
    double lat = 0.0;
    double lon = 0.0;
};
