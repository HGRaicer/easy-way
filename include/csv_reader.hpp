#pragma once
#include <string>
#include <vector>
#include <cstdint>

// Структура для представления точки TSP
struct TspPoint {
    int user_id;
    double lat;
    double lon;
};

/**
 * @brief Загружает точки TSP из CSV файла.
 * Поддерживает форматы: "lat,lon" и "id,lat,lon".
 * * @param csv_path Путь к исходному CSV файлу.
 * @param points Вектор для сохранения считанных точек.
 * @param error Строка для записи сообщения об ошибке в случае неудачи.
 * @return true Если загрузка прошла успешно.
 * @return false Если произошла ошибка (например, файл не найден).
 */
bool load_tsp_points_from_csv(const std::string& csv_path,
    std::vector<TspPoint>& points,
    std::string& error);

/**
 * @brief Записывает координаты точек в строку через пробел в файл.
 * Формат вывода: lat1 lon1 lat2 lon2 ...
 * * @param output_path Путь к выходному файлу.
 * @param points Вектор точек для записи.
 */
void write_tsp_input_file(const std::string& output_path,
    const std::vector<TspPoint>& points);

/**
 * @brief Сохраняет маппинг между пользовательскими ID и внутренними плотными ID.
 * Создает CSV файл со структурой: user_id,dense_id,lat,lon
 * * @param points Исходный вектор точек.
 * @param dense_ids Вектор соответствующих плотных (внутренних) идентификаторов.
 * @param output_path Путь для сохранения результирующего CSV.
 * @return true Если файл успешно сохранен.
 * @return false Если размеры векторов не совпадают или файл не может быть открыт.
 */
bool save_id_mapping(const std::vector<TspPoint>& points,
    const std::vector<uint32_t>& dense_ids,
    const std::string& output_path);
