#pragma once

#include "search.h"
#include <string>
#include <vector>
#include <cstdint>

/**
 * Решает задачу коммивояжёра (TSP) для заданных координат.
 *
 * @param inputTxtPath путь к файлу с координатами (одна строка: lat1 lon1 lat2 lon2 ...)
 * @param graphBinPath путь к graph.bin (бинарный граф дорог)
 * @param outputPath путь для сохранения результата
 * @param mapCsvPath путь к id_map.csv (для геокодирования)
 * @param full_output если true, выводит полные маршруты между точками
 * @param metric метрика (Distance или Time)
 * @param error строка для сообщения об ошибке
 * @return true если успешно, false если ошибка
 */
bool solve_tsp(const std::string& inputTxtPath,
    const std::string& graphBinPath,
    const std::string& outputPath,
    const std::string& mapCsvPath,
    bool full_output,
    SearchMetric metric,
    std::string& error);