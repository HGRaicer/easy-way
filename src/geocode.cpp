#include "geocode.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>

struct MapPoint {
    int dense_id;
    long long osm_id;
    double lat;
    double lon;
};

static double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371.0;
    const double PI = 3.14159265358979323846;

    double dlat = (lat2 - lat1) * PI / 180.0;
    double dlon = (lon2 - lon1) * PI / 180.0;

    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
        std::cos(lat1 * PI / 180.0) * std::cos(lat2 * PI / 180.0) *
        std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    return R * c;
}

static bool loadPoints(const std::string& filename, std::vector<MapPoint>& points, std::string& error) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        error = "Cannot open: " + filename;
        return false;
    }

    std::string line;
    // skip header
    if (!std::getline(file, line)) {
        error = "Empty file";
        return false;
    }

    points.clear();
    int lineNum = 2;

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        MapPoint p;
        char comma;

        if (!(ss >> p.dense_id >> comma >> p.osm_id >> comma >> p.lat >> comma >> p.lon)) {
            error = "Parse error at line " + std::to_string(lineNum) + ": " + line;
            return false;
        }

        points.push_back(p);
        lineNum++;
    }

    return true;
}

static int findNearestId(const std::vector<MapPoint>& points, double targetLat, double targetLon) {
    if (points.empty()) return -1;

    int bestId = points[0].dense_id;
    double bestDist = haversineDistance(targetLat, targetLon, points[0].lat, points[0].lon);

    for (const auto& p : points) {
        double dist = haversineDistance(targetLat, targetLon, p.lat, p.lon);
        if (dist < bestDist) {
            bestDist = dist;
            bestId = p.dense_id;
        }
    }

    return bestId;
}

bool convert_coordinates_to_ids(const std::string& mapCsvPath,
    const std::string& inputTxtPath,
    const std::string& outputQueriesPath,
    std::string& error) {

    // 1. Load points from CSV
    std::vector<MapPoint> points;
    if (!loadPoints(mapCsvPath, points, error)) {
        return false;
    }

    std::cerr << "Loaded " << points.size() << " points from CSV" << std::endl;

    // 2. Open input.txt
    std::ifstream inputFile(inputTxtPath);
    if (!inputFile.is_open()) {
        error = "Cannot open input file: " + inputTxtPath;
        return false;
    }

    // 3. Open output (queries.txt)
    std::ofstream outputFile(outputQueriesPath);
    if (!outputFile.is_open()) {
        error = "Cannot open output file: " + outputQueriesPath;
        return false;
    }

    // 4. Process each line
    std::string line;
    int lineNum = 1;
    int convertedCount = 0;

    while (std::getline(inputFile, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::stringstream ss(line);
        double lat1, lon1, lat2, lon2;

        if (!(ss >> lat1 >> lon1 >> lat2 >> lon2)) {
            error = "Invalid format at line " + std::to_string(lineNum) +
                ". Expected: lat1 lon1 lat2 lon2";
            return false;
        }

        int id1 = findNearestId(points, lat1, lon1);
        int id2 = findNearestId(points, lat2, lon2);

        if (id1 == -1 || id2 == -1) {
            error = "No points found for line " + std::to_string(lineNum);
            return false;
        }

        outputFile << id1 << " " << id2 << std::endl;

        std::cerr << "Line " << lineNum << ": (" << lat1 << "," << lon1 << ") -> id " << id1
            << ", (" << lat2 << "," << lon2 << ") -> id " << id2 << std::endl;

        lineNum++;
        convertedCount++;
    }

    std::cerr << "Converted " << convertedCount << " pairs to " << outputQueriesPath << std::endl;

    return true;
}