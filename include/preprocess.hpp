#pragma once
#include <string>

bool preprocess_pbf_to_bin(const std::string& pbf_path,
    const std::string& out_bin_path,
    const std::string& out_map_csv_path,
    std::string& err);
