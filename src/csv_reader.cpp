#include "csv_reader.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <iomanip>

bool load_tsp_points_from_csv(const std::string& csv_path,
    std::vector<TspPoint>& points,
    std::string& error) {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        error = "Cannot open CSV: " + csv_path;
        return false;
    }

    std::string line;
    // ѕропускаем заголовок (если есть)
    std::getline(file, line);

    points.clear();
    int line_num = 2;
    int auto_id = 1;

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        TspPoint p;
        std::string token;

        // ѕробуем определить формат: id,lat,lon или lat,lon
        std::getline(ss, token, ',');

        // ѕровер€ем, €вл€етс€ ли первое поле числом с плавающей точкой (широтой)
        bool is_lat = true;
        int dot_count = 0;
        for (char c : token) {
            if (c == '.') dot_count++;
            if (c != '.' && c != '-' && (c < '0' || c > '9')) {
                is_lat = false;
                break;
            }
        }

        if (is_lat && dot_count == 1) {
            // ‘ормат: lat,lon (без id)
            ss.clear();
            ss.str(line);
            p.user_id = auto_id++;
            ss >> p.lat;
            ss.ignore();  // пропускаем зап€тую
            ss >> p.lon;
        }
        else {
            // ‘ормат: id,lat,lon
            p.user_id = std::stoi(token);
            std::getline(ss, token, ','); p.lat = std::stod(token);
            std::getline(ss, token, ','); p.lon = std::stod(token);
        }

        points.push_back(p);
        line_num++;
    }

    if (points.empty()) {
        error = "No points loaded from CSV";
        return false;
    }

    std::cerr << "Loaded " << points.size() << " points from CSV" << std::endl;
    for (const auto& p : points) {
        std::cerr << "  User point " << p.user_id << ": (" << p.lat << ", " << p.lon << ")" << std::endl;
    }

    return true;
}

void write_tsp_input_file(const std::string& output_path,
    const std::vector<TspPoint>& points) {
    std::ofstream out(output_path);
    if (!out.is_open()) {
        std::cerr << "Warning: cannot write to " << output_path << std::endl;
        return;
    }

    // «аписываем только координаты (без id)
    for (const auto& p : points) {
        out << p.lat << " " << p.lon << " ";
    }
    out << std::endl;

    std::cerr << "TSP input file written: " << output_path << std::endl;
}

bool save_id_mapping(const std::vector<TspPoint>& points,
    const std::vector<uint32_t>& dense_ids,
    const std::string& output_path) {
    if (points.size() != dense_ids.size()) {
        return false;
    }

    std::ofstream out(output_path);
    if (!out.is_open()) {
        return false;
    }

    out << "user_id,dense_id,lat,lon" << std::endl;
    for (size_t i = 0; i < points.size(); ++i) {
        out << points[i].user_id << "," << dense_ids[i] << ","
            << points[i].lat << "," << points[i].lon << std::endl;
    }
    return true;
}

bool parse_coordinate_pairs_csv_to_txt(const std::string& csv_path,
    const std::string& txt_out_path,
    std::string& error_msg) {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        error_msg = "Cannot open CSV file: " + csv_path;
        return false;
    }

    std::vector<std::string> queries;
    std::string line;

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        // «амен€ем попул€рные разделители на пробелы
        std::replace(line.begin(), line.end(), ',', ' ');
        std::replace(line.begin(), line.end(), ';', ' ');

        std::istringstream iss(line);
        double lat1, lon1, lat2, lon2;

        // ≈сли удалось считать 4 числа подр€д Ч сохран€ем их
        if (iss >> lat1 >> lon1 >> lat2 >> lon2) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(6)
                << lat1 << " " << lon1 << " " << lat2 << " " << lon2;
            queries.push_back(oss.str());
        }
    }

    if (queries.empty()) {
        error_msg = "No valid coordinate pairs (lat1, lon1, lat2, lon2) found in CSV.";
        return false;
    }

    std::ofstream out(txt_out_path);
    if (!out.is_open()) {
        error_msg = "Cannot create temp txt file: " + txt_out_path;
        return false;
    }

    // «аписываем собранные запросы, раздел€€ переносом строки
    for (size_t i = 0; i < queries.size(); ++i) {
        out << queries[i];
        if (i + 1 < queries.size()) {
            out << "\n";
        }
    }

    return true;
}