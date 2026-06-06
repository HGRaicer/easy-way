#include "tsp.hpp"
#include "geocode.hpp"
#include "search.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>
#include <cstdio>
#include <memory>
#include <cstring>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace operations_research {
    void SolveTSP(const std::vector<std::vector<int64_t>>& dist_matrix,
        std::vector<int>& best_order,
        int64_t& best_cost);
}

static bool parse_coordinates(const std::string& line,
    std::vector<std::pair<double, double>>& coords,
    std::string& error) {
    std::stringstream ss(line);
    std::vector<double> numbers;
    double num;
    while (ss >> num) { numbers.push_back(num); }

    if (numbers.size() < 2) { error = "At least one coordinate pair required"; return false; }
    if (numbers.size() % 2 != 0) { error = "Odd number of coordinates. Expected pairs of lat lon"; return false; }

    coords.clear();
    for (size_t i = 0; i < numbers.size(); i += 2) {
        coords.push_back({ numbers[i], numbers[i + 1] });
    }
    return true;
}

namespace operations_research {
    void SolveTSP(const std::vector<std::vector<int64_t>>& dist_matrix,
        std::vector<int>& best_order,
        int64_t& best_cost) {
        const int num_nodes = (int)dist_matrix.size();

        if (num_nodes < 2) {
            best_order = { 0 };
            best_cost = 0;
            return;
        }

        RoutingIndexManager manager(num_nodes, 1, RoutingNodeIndex(0));
        RoutingModel routing(manager);

        const int transit_callback_index = routing.RegisterTransitCallback(
            [&dist_matrix, &manager](int64_t from_index, int64_t to_index) -> int64_t {
                int from_node = manager.IndexToNode(from_index).value();
                int to_node = manager.IndexToNode(to_index).value();
                return dist_matrix[from_node][to_node];
            }
        );

        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

        RoutingSearchParameters search_parameters = operations_research::DefaultRoutingSearchParameters();
        search_parameters.set_first_solution_strategy(operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC);
        search_parameters.set_local_search_metaheuristic(operations_research::LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
        search_parameters.mutable_time_limit()->set_seconds(2);

        const Assignment* solution = routing.SolveWithParameters(search_parameters);

        if (solution) {
            best_cost = solution->ObjectiveValue();
            best_order.clear();
            int64_t index = routing.Start(0);
            while (!routing.IsEnd(index)) {
                int node = manager.IndexToNode(index).value();
                best_order.push_back(node);
                index = solution->Value(routing.NextVar(index));
            }
            best_order.push_back(best_order[0]); // Возврат в исходную точку
        }
        else {
            best_cost = -1;
            best_order.clear();
        }
    }
}

bool solve_tsp(const std::string& inputTxtPath,
    const std::string& graphBinPath,
    const std::string& outputPath,
    const std::string& mapCsvPath,
    bool full_output,
    SearchMetric metric,
    std::string& error) {

    std::ifstream inputFile(inputTxtPath);
    if (!inputFile.is_open()) { error = "Cannot open input file: " + inputTxtPath; return false; }
    std::string line;
    std::getline(inputFile, line);
    inputFile.close();

    std::vector<std::pair<double, double>> coords;
    if (!parse_coordinates(line, coords, error)) return false;

    int n = (int)coords.size();
    if (n < 2) { error = "At least 2 points required for TSP"; return false; }

    std::vector<uint32_t> node_ids;
    if (!operations_research::geocode_vector_of_coordinates(mapCsvPath, coords, node_ids, error)) return false;

    // === ШАГ 1: Генерируем матрицу смежности ===
    FILE* graph_file = fopen(graphBinPath.c_str(), "rb");
    if (!graph_file) { error = "Cannot open graph file: " + graphBinPath; return false; }

    std::string tempAllInput = "temp_all_pairs_input.txt";
    std::string tempAllOutput = "temp_all_pairs_output.txt";

    std::ofstream all_in(tempAllInput);
    if (!all_in) { fclose(graph_file); error = "Cannot create batch input file"; return false; }

    // Пишем в файл запросы ТОЛЬКО для разных точек
    int expected_lines = 0;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == j) continue;
            all_in << node_ids[i] << " " << node_ids[j] << "\n";
            expected_lines++;
        }
    }
    all_in.close();

    FILE* inf = fopen(tempAllInput.c_str(), "r");
    FILE* outf = fopen(tempAllOutput.c_str(), "w");
    if (!inf || !outf) {
        if (inf) fclose(inf); if (outf) fclose(outf);
        fclose(graph_file); std::remove(tempAllInput.c_str());
        error = "Cannot open batch IO files"; return false;
    }

    fseek(graph_file, 0, SEEK_SET);
    // Запускаем run_search с false, так как геометрия на первом шаге не нужна
    run_search(graph_file, inf, outf, false, metric);

    fclose(inf);
    fclose(outf);
    fclose(graph_file);

    // Считываем результаты
    std::ifstream all_out(tempAllOutput);
    if (!all_out) {
        std::remove(tempAllInput.c_str()); std::remove(tempAllOutput.c_str());
        error = "Cannot open batch output for reading"; return false;
    }

    std::vector<std::vector<int64_t>> time_matrix(n, std::vector<int64_t>(n, 0));
    std::vector<std::vector<int64_t>> dist_matrix(n, std::vector<int64_t>(n, 0));

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == j) {
                // Диагональ — это точка сама в себя, веса равны 0
                time_matrix[i][j] = 0;
                dist_matrix[i][j] = 0;
                continue;
            }

            std::string raw_route_line;
            if (!std::getline(all_out, raw_route_line)) {
                all_out.close(); std::remove(tempAllInput.c_str()); std::remove(tempAllOutput.c_str());
                error = "Batch output ended unexpectedly at cell [" + std::to_string(i) + "][" + std::to_string(j) + "]";
                return false;
            }
            if (raw_route_line[0] == '=')
                if (!std::getline(all_out, raw_route_line)) {
                    all_out.close(); std::remove(tempAllInput.c_str()); std::remove(tempAllOutput.c_str());
                    error = "Batch output ended unexpectedly at cell [" + std::to_string(i) + "][" + std::to_string(j) + "]";
                    return false;
                }
            std::stringstream ss(raw_route_line);
            double time_val = 0.0, dist_val = 0.0;
            uint32_t nodes_cnt = 0;

            if (!(ss >> time_val >> dist_val)) {
                all_out.close(); std::remove(tempAllInput.c_str()); std::remove(tempAllOutput.c_str());
                error = "Failed to parse search metrics at [" + std::to_string(i) + "][" + std::to_string(j) + "]. Line: " + raw_route_line;
                return false;
            }

            // Переводим в миллиединицы для OR-Tools (целочисленная арифметика)
            time_matrix[i][j] = static_cast<int64_t>(time_val * 1000.0);
            dist_matrix[i][j] = static_cast<int64_t>(dist_val * 1000.0);
        }
    }
    all_out.close();

    std::remove(tempAllInput.c_str());
    std::remove(tempAllOutput.c_str());

    // === ШАГ 2: Вызов оптимизатора OR-Tools ===
    const auto& target_matrix = (metric == SearchMetric::Time) ? time_matrix : dist_matrix;

    std::vector<int> best_order;
    int64_t best_cost;
    operations_research::SolveTSP(target_matrix, best_order, best_cost);

    if (best_cost < 0 || best_order.empty()) { error = "TSP solver failed to find a solution"; return false; }

    // Расчет итоговых метрик по выбранному контуру
    int64_t total_time_scaled = 0;
    int64_t total_dist_scaled = 0;
    for (size_t i = 0; i < best_order.size() - 1; ++i) {
        int u = best_order[i];
        int v = best_order[i + 1];
        total_time_scaled += time_matrix[u][v];
        total_dist_scaled += dist_matrix[u][v];
    }

    // Сохраняем текстовый манифест решения
    std::ofstream outputFile(outputPath);
    if (!outputFile.is_open()) { error = "Cannot open output file: " + outputPath; return false; }

    outputFile << "=== TSP Optimization Result ===\n";
    outputFile << "Optimized by: " << (metric == SearchMetric::Time ? "TIME" : "DISTANCE") << "\n";
    outputFile << "Total Time (seconds): " << (total_time_scaled / 1000.0) << "\n";
    outputFile << "Total Distance (meters): " << (total_dist_scaled / 1000.0) << "\n\n";
    outputFile << "Optimal order (starting from base):\n";

    for (size_t k = 0; k < best_order.size() - 1; ++k) {
        int idx = best_order[k];
        outputFile << "  " << (k + 1) << ". Point " << (idx + 1) << ": "
            << coords[idx].first << ", " << coords[idx].second << " -> node_id " << node_ids[idx];
        if (idx == 0) outputFile << " (BASE)";
        outputFile << "\n";
    }
    outputFile << "  Return to base (Point 1)\n";

    // === ШАГ 3: Точечный сбор геометрии ТОЛЬКО для N выигравших перегонов ===
    if (full_output) {
        outputFile << "\n=== Full routes between points ===\n";

        FILE* graph_file2 = fopen(graphBinPath.c_str(), "rb");
        if (!graph_file2) {
            outputFile << "\n[Error] Cannot re-open graph file to fetch geometry.\n";
            outputFile.close();
            return true;
        }

        std::string tempFinalInput = "temp_final_route_input.txt";
        std::string tempFinalOutput = "temp_final_route_output.txt";

        std::ofstream final_in(tempFinalInput);
        for (size_t k = 0; k < best_order.size() - 1; ++k) {
            int from_idx = best_order[k];
            int to_idx = best_order[k + 1];
            final_in << node_ids[from_idx] << " " << node_ids[to_idx] << "\n";
        }
        final_in.close();

        FILE* finf = fopen(tempFinalInput.c_str(), "r");
        FILE* foutf = fopen(tempFinalOutput.c_str(), "w");
        if (finf && foutf) {
            fseek(graph_file2, 0, SEEK_SET);
            // Вызываем run_search с флагом true — теперь она допишет цепочки ID
            run_search(graph_file2, finf, foutf, true, metric);
            fclose(finf); fclose(foutf);

            std::ifstream final_out(tempFinalOutput);
            std::string route_geometry_line;

            for (size_t k = 0; k < best_order.size() - 1; ++k) {
                int from_idx = best_order[k];
                int to_idx = best_order[k + 1];

                outputFile << "\nRoute from Point " << (from_idx + 1) << " to Point " << (to_idx + 1)
                    << " (Time: " << (time_matrix[from_idx][to_idx] / 1000.0)
                    << "s, Dist: " << (dist_matrix[from_idx][to_idx] / 1000.0) << "m):\n";

                if (std::getline(final_out, route_geometry_line)) {
                    if (route_geometry_line[0] == '=')
                        std::getline(final_out, route_geometry_line);
                    std::stringstream ss(route_geometry_line);
                    double dump_t, dump_d; uint32_t dump_c;
                    // Считываем первые три обязательных числа
                    ss >> dump_t >> dump_d >> dump_c;

                    // Всё, что осталось в строке — это исключительно список ID узлов!
                    std::string only_nodes;
                    std::getline(ss, only_nodes);
                    outputFile << " " << only_nodes << "\n";
                }
                else {
                    outputFile << "  [No geometry found]\n";
                }
            }
            final_out.close();
        }

        if (graph_file2) fclose(graph_file2);
        std::remove(tempFinalInput.c_str());
        std::remove(tempFinalOutput.c_str());
    }

    outputFile.close();
    return true;
}