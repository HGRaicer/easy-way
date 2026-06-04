#include "geocode.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <vector>
#include <string>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// Используем geographic координатную систему (долгота, широта)
using BoostPoint = bg::model::point<double, 2, bg::cs::geographic<bg::degree>>;
using RtreeValue = std::pair<BoostPoint, uint32_t>;

namespace operations_research {

    bool geocode_vector_of_coordinates(const std::string& mapCsvPath,
        const std::vector<std::pair<double, double>>& coords,
        std::vector<uint32_t>& output_node_ids,
        std::string& error) {

        size_t n = coords.size();
        if (n == 0) {
            output_node_ids.clear();
            return true;
        }
        output_node_ids.assign(n, 0);

        std::ifstream csvFile(mapCsvPath);
        if (!csvFile.is_open()) {
            error = "Geocode Error: Cannot open map CSV file: " + mapCsvPath;
            return false;
        }

        std::string line;
        std::getline(csvFile, line); // Пропускаем заголовок

        std::vector<RtreeValue> rtree_elements;
        rtree_elements.reserve(9500000);

        while (std::getline(csvFile, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            int dense_id; long long osm_id; double n_lat, n_lon; char c;
            if (ss >> dense_id >> c >> osm_id >> c >> n_lat >> c >> n_lon) {
                // Boost.Geometry требует порядок (Долгота, Широта)
                rtree_elements.push_back(std::make_pair(BoostPoint(n_lon, n_lat), static_cast<uint32_t>(dense_id)));
            }
        }
        csvFile.close();

        std::cerr << "[Geocode Engine] Loaded " << rtree_elements.size()
            << " nodes from CSV. Building R-tree index..." << std::endl;

        // Быстрое построение R-дерева методом OMT bulk-loading
        bgi::rtree<RtreeValue, bgi::quadratic<16>> rtree(rtree_elements.begin(), rtree_elements.end());

        rtree_elements.clear();
        rtree_elements.shrink_to_fit();

        std::cerr << "[Geocode Engine] R-tree built successfully. Starting spatial queries..." << std::endl;

#pragma omp parallel for schedule(dynamic)
        for (int i = 0; i < (int)n; ++i) {
            double query_lat = coords[i].first;
            double query_lon = coords[i].second;

            BoostPoint query_point(query_lon, query_lat);
            std::vector<RtreeValue> result_nodes;

            // Исправлено: bgi::index::nearest вместо некорректного bgi::queries::nearest
            rtree.query(bgi::nearest(query_point, 1), std::back_inserter(result_nodes));

            if (!result_nodes.empty()) {
                output_node_ids[i] = result_nodes[0].second;
            }
            else {
                output_node_ids[i] = 0;
            }
        }

        std::cerr << "[Geocode Engine] Spatial matching completed for " << n << " points." << std::endl;
        return true;
    }

    bool convert_coordinates_to_ids(const std::string& mapCsvPath,
        const std::string& inputTxtPath,
        const std::string& outputQueriesPath,
        std::string& error) {

        std::ifstream inputFile(inputTxtPath);
        if (!inputFile.is_open()) {
            error = "Cannot open input file: " + inputTxtPath;
            return false;
        }

        struct QueryPair { std::pair<double, double> p1; std::pair<double, double> p2; };
        std::vector<QueryPair> queryPairs;
        std::vector<std::pair<double, double>> flat_coords;

        std::string line;
        while (std::getline(inputFile, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            double lat1, lon1, lat2, lon2;
            if (!(ss >> lat1 >> lon1 >> lat2 >> lon2)) {
                error = "Invalid format in input file";
                return false;
            }
            queryPairs.push_back({ {lat1, lon1}, {lat2, lon2} });
            flat_coords.push_back({ lat1, lon1 });
            flat_coords.push_back({ lat2, lon2 });
        }
        inputFile.close();

        std::vector<uint32_t> flat_ids;
        if (!geocode_vector_of_coordinates(mapCsvPath, flat_coords, flat_ids, error)) {
            return false;
        }

        std::ofstream outputFile(outputQueriesPath);
        if (!outputFile.is_open()) {
            error = "Cannot open output file: " + outputQueriesPath;
            return false;
        }

        size_t id_idx = 0;
        for (size_t i = 0; i < queryPairs.size(); ++i) {
            uint32_t id1 = flat_ids[id_idx++];
            uint32_t id2 = flat_ids[id_idx++];
            outputFile << id1 << " " << id2 << "\n";
        }
        outputFile.close();

        return true;
    }

} // namespace operations_research