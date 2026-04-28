#ifndef GEOCODE_HPP
#define GEOCODE_HPP

#include <string>

bool convert_coordinates_to_ids(const std::string& mapCsvPath,
    const std::string& inputTxtPath,
    const std::string& outputQueriesPath,
    std::string& error);

#endif // GEOCODE_HPP