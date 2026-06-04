#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace operations_research {

    // Сверхбыстрый геокодер на базе Boost.Geometry R-tree
    bool geocode_vector_of_coordinates(const std::string& mapCsvPath,
        const std::vector<std::pair<double, double>>& coords,
        std::vector<uint32_t>& output_node_ids,
        std::string& error);

    // Старый файловый интерфейс запросов (сохранен для обратной совместимости)
    bool convert_coordinates_to_ids(const std::string& mapCsvPath,
        const std::string& inputTxtPath,
        const std::string& outputQueriesPath,
        std::string& error);

} // namespace operations_research