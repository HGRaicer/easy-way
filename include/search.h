#pragma once
#include <cstdio>

enum class SearchMetric {
    Distance,
    Time
};

void run_search(FILE* graph_file,
    FILE* input_file,
    FILE* output_file,
    bool full_output,
    SearchMetric metric);