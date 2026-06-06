#pragma once

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QSplitter>
#include <QtCore/QFutureWatcher>

#include "visualizer.hpp"
#include "search.h"

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void onBrowseMapClicked();
    void onLoadMapClicked();
    void onSearchSettingsClicked();
    void onSearchClicked();

    void onMapLoadFinished();
    void onSearchFinished();

private:
    void setupUi();
    void logMessage(const QString& msg);

    void loadSettings();
    void saveSettings();

    // Элементы UI
    QPushButton* btnLoadMap;
    QToolButton* btnBrowseMap;

    QPushButton* btnSearch;
    QToolButton* btnSettings;

    QComboBox* cbAlgorithm;
    QTextEdit* txtLog;

    // Визуализатор
    GraphVisualizer viz;
    QWidget* mapWidget;

    // Фоновые задачи
    QFutureWatcher<bool> mapWatcher;
    QFutureWatcher<bool> searchWatcher;

    // Пути к картам
    QString pbfPath;
    QString graphBinPath;
    QString mapCsvPath;

    // Настройки поиска
    QString astarInput;
    QString astarOutput = "astar_result.txt";

    QString tspInputCsv;
    QString tspOutput = "tsp_result.txt";

    QString vrpFleetCsv;
    QString vrpOrdersCsv;
    QString vrpOutput = "vrp_result.txt";

    bool useFullOutput = true;
    SearchMetric currentMetric = SearchMetric::Time;
};