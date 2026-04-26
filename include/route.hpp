#pragma once

#include <cstdint>
#include <string>
#include <vector>

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
};

struct Waypoint {
    double lat = 0.0;
    double lon = 0.0;
};
