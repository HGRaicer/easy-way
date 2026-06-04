#pragma once

#include <cstdio>
#include <string>

enum class SearchMetric {
    Time,
    Distance
};

void run_search(FILE* graph_file,
    FILE* input_file,
    FILE* output_file,
    bool full_output,
    SearchMetric metric);