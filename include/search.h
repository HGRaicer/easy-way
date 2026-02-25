#pragma once
#include <vector>
#include <queue>
#include <cstdint>
#include <cstdio>

struct SearchNode {
    uint32_t id;
    double lat, lon;
};

struct PathEntry {
    uint32_t node;
    double cost;
    bool operator>(const PathEntry& other) const noexcept {
        return cost > other.cost || (cost == other.cost && node > other.node);
    }
};

void run_search(FILE* graph_file, FILE* input_file, FILE* output_file, bool full_output);
