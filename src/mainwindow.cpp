#include "mainwindow.hpp"
#include "preprocess.hpp"
#include "tsp.hpp"
#include "vrp_solver.hpp"
#include "search.h"
#include "route.hpp"
#include "geocode.hpp"
#include "csv_reader.hpp" // Требуется для load_tsp_points_from_csv

#include <QtWidgets/QFileDialog>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QMessageBox>
#include <QtConcurrent/QtConcurrent>
#include <QtCore/QFileInfo>
#include <QtCore/QDir>
#include <QtCore/QSettings>
#include <QtCore/QFileInfo>

// Прототип функции для работы A* (аналог из main.cpp)
extern void run_search(FILE* graph_file, FILE* input_file, FILE* output_file, bool full_output, SearchMetric metric);

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setupUi();

    // Подключаем сигналы завершения фоновых потоков к слотам UI
    connect(&mapWatcher, &QFutureWatcher<bool>::finished, this, &MainWindow::onMapLoadFinished);
    connect(&searchWatcher, &QFutureWatcher<bool>::finished, this, &MainWindow::onSearchFinished);

    loadSettings();
}

MainWindow::~MainWindow() {
    saveSettings();
}

void MainWindow::loadSettings() {
    // "MyCompany" и "GraphOptimizer" — это ключи в реестре / папки для хранения конфига
    QSettings settings("MyCompany", "GraphOptimizer");

    // Восстанавливаем пути к картам
    pbfPath = settings.value("paths/pbfPath").toString();
    graphBinPath = settings.value("paths/graphBinPath").toString();
    mapCsvPath = settings.value("paths/mapCsvPath").toString();

    // Заодно восстановим пути к файлам поиска, чтобы пользователю не вводить их заново
    astarInput = settings.value("search/astarInput").toString();
    astarOutput = settings.value("search/astarOutput", "astar_result.txt").toString();
    tspInputCsv = settings.value("search/tspInputCsv").toString();
    tspOutput = settings.value("search/tspOutput", "tsp_result.txt").toString();
    vrpFleetCsv = settings.value("search/vrpFleetCsv").toString();
    vrpOrdersCsv = settings.value("search/vrpOrdersCsv").toString();
    vrpOutput = settings.value("search/vrpOutput", "vrp_result.txt").toString();
    useFullOutput = settings.value("search/useFullOutput", true).toBool();

    // ГЛАВНАЯ ЛОГИКА: Если бинарный граф уже существует на диске, 
    // подгружаем его автоматически без повторного препроцессинга!
    if (!graphBinPath.isEmpty() && QFileInfo::exists(graphBinPath)) {
        logMessage("Обнаружен готовый препроцессинг графа с прошлого запуска. Загрузка...");

        // Загружаем бинарник в граф
        viz.load(graphBinPath.toStdString());

        // Пересоздаем виджет карты на лету и меняем заглушку
        QSplitter* splitter = qobject_cast<QSplitter*>(centralWidget());
        if (splitter) {
            QWidget* oldWidget = splitter->widget(1);
            mapWidget = viz.createWidget(this);
            if (mapWidget) {
                oldWidget->deleteLater();
                splitter->addWidget(mapWidget);
                viz.updateVisuals(); // Отрисовываем карту на экране
                logMessage("Карта успешно восстановлена и готова к работе!");
            }
        }
    }
    else if (!pbfPath.isEmpty()) {
        logMessage("Найден путь к карте: " + QFileInfo(pbfPath).fileName() + ", но бинарный граф не сгенерирован. Нажмите 'Загрузить карту'.");
    }
}

void MainWindow::saveSettings() {
    QSettings settings("MyCompany", "GraphOptimizer");

    // Записываем все текущие пути в хранилище
    settings.setValue("paths/pbfPath", pbfPath);
    settings.setValue("paths/graphBinPath", graphBinPath);
    settings.setValue("paths/mapCsvPath", mapCsvPath);

    settings.setValue("search/astarInput", astarInput);
    settings.setValue("search/astarOutput", astarOutput);
    settings.setValue("search/tspInputCsv", tspInputCsv);
    settings.setValue("search/tspOutput", tspOutput);
    settings.setValue("search/vrpFleetCsv", vrpFleetCsv);
    settings.setValue("search/vrpOrdersCsv", vrpOrdersCsv);
    settings.setValue("search/vrpOutput", vrpOutput);
    settings.setValue("search/useFullOutput", useFullOutput);
}

void MainWindow::setupUi() {
    setWindowTitle("easy-way");
    resize(1200, 800);

    QSplitter* splitter = new QSplitter(Qt::Horizontal, this);
    setCentralWidget(splitter);

    // --- ЛЕВАЯ ПАНЕЛЬ (Меню) ---
    QWidget* leftPanel = new QWidget(this);
    QVBoxLayout* leftLayout = new QVBoxLayout(leftPanel);
    leftLayout->setAlignment(Qt::AlignTop);
    leftPanel->setMaximumWidth(350);

    // 1. Блок Загрузки карты
    QHBoxLayout* mapLayout = new QHBoxLayout();
    btnLoadMap = new QPushButton("Загрузить карту", this);
    btnBrowseMap = new QToolButton(this);
    btnBrowseMap->setText("📁");
    btnBrowseMap->setToolTip("Указать путь к .pbf файлу");
    mapLayout->addWidget(btnLoadMap, 1);
    mapLayout->addWidget(btnBrowseMap, 0);

    // 2. Блок Поиска
    QHBoxLayout* searchLayout = new QHBoxLayout();
    btnSearch = new QPushButton("Поиск", this);
    btnSettings = new QToolButton(this);
    btnSettings->setText("⚙");
    btnSettings->setToolTip("Настройки поиска");
    searchLayout->addWidget(btnSearch, 1);
    searchLayout->addWidget(btnSettings, 0);

    // 3. Выбор алгоритма
    cbAlgorithm = new QComboBox(this);
    cbAlgorithm->addItem("A* (Кратчайший путь)");
    cbAlgorithm->addItem("TSP (Задача коммивояжера)");
    cbAlgorithm->addItem("VRP (Маршрутизация транспорта)");

    // Лог
    txtLog = new QTextEdit(this);
    txtLog->setReadOnly(true);

    leftLayout->addLayout(mapLayout);
    leftLayout->addLayout(searchLayout);
    leftLayout->addWidget(cbAlgorithm);
    leftLayout->addWidget(txtLog);

    // --- ПРАВАЯ ПАНЕЛЬ (Карта) ---
    mapWidget = viz.createWidget(this);
    if (!mapWidget) {
        // Если граф еще не загружен, создаем пустой виджет-заглушку
        mapWidget = new QWidget(this);
        mapWidget->setStyleSheet("background-color: #0f0f12;");
    }

    splitter->addWidget(leftPanel);
    splitter->addWidget(mapWidget);
    splitter->setStretchFactor(1, 1); // Карта занимает основное место

    // Подключение слотов
    connect(btnBrowseMap, &QToolButton::clicked, this, &MainWindow::onBrowseMapClicked);
    connect(btnLoadMap, &QPushButton::clicked, this, &MainWindow::onLoadMapClicked);
    connect(btnSettings, &QToolButton::clicked, this, &MainWindow::onSearchSettingsClicked);
    connect(btnSearch, &QPushButton::clicked, this, &MainWindow::onSearchClicked);
}

void MainWindow::logMessage(const QString& msg) {
    txtLog->append(msg);
}

void MainWindow::onBrowseMapClicked() {
    QString path = QFileDialog::getOpenFileName(this, "Выберите PBF файл", "", "OSM PBF Files (*.osm.pbf);;All Files (*)");
    if (!path.isEmpty()) {
        pbfPath = path;

        // Автоматически назначаем выходные файлы в ту же папку
        QFileInfo fi(pbfPath);
        graphBinPath = fi.absolutePath() + "/graph.bin";
        mapCsvPath = fi.absolutePath() + "/id_map.csv";

        logMessage("Выбран файл карты: " + fi.fileName());
    }
}

void MainWindow::onLoadMapClicked() {
    if (pbfPath.isEmpty()) {
        QMessageBox::warning(this, "Ошибка", "Сначала выберите файл .pbf с помощью кнопки с папкой!");
        return;
    }

    btnLoadMap->setEnabled(false);
    btnSearch->setEnabled(false);
    logMessage("Начинаем препроцессинг карты. Пожалуйста, подождите...");

    // Асинхронный запуск
    QFuture<bool> future = QtConcurrent::run([=]() {
        std::string err;
        return preprocess_pbf_to_bin(pbfPath.toStdString(), graphBinPath.toStdString(), mapCsvPath.toStdString(), err);
        });
    mapWatcher.setFuture(future);
}

void MainWindow::onMapLoadFinished() {
    btnLoadMap->setEnabled(true);
    btnSearch->setEnabled(true);

    if (mapWatcher.result()) {
        logMessage("Карта успешно загружена и обработана!");

        viz.load(graphBinPath.toStdString());

        QSplitter* splitter = qobject_cast<QSplitter*>(centralWidget());
        QWidget* oldWidget = splitter->widget(1);
        mapWidget = viz.createWidget(this);
        oldWidget->deleteLater();
        splitter->addWidget(mapWidget);

        viz.updateVisuals();

        // Сразу сохраняем пути в кэш настроек
        saveSettings();
    }
    else {
        logMessage("Ошибка при загрузке карты.");
    }
}

void MainWindow::onSearchSettingsClicked() {
    QDialog dlg(this);
    dlg.setWindowTitle("Настройки поиска: " + cbAlgorithm->currentText());
    QFormLayout layout(&dlg);

    int algo = cbAlgorithm->currentIndex();

    // Лямбда для создания поля выбора файла
    auto createFileInput = [&](const QString& label, QString& pathVar, const QString& filter, bool isSave) {
        QHBoxLayout* hl = new QHBoxLayout();
        QLineEdit* le = new QLineEdit(pathVar);
        QToolButton* tb = new QToolButton();
        tb->setText("...");
        hl->addWidget(le);
        hl->addWidget(tb);
        layout.addRow(label, hl);

        connect(tb, &QToolButton::clicked, [&, le, filter, isSave]() {
            QString selected;
            if (isSave) selected = QFileDialog::getSaveFileName(&dlg, label, le->text(), filter);
            else selected = QFileDialog::getOpenFileName(&dlg, label, le->text(), filter);

            if (!selected.isEmpty()) {
                le->setText(selected);
                pathVar = selected;
            }
            });

        // Обновляем переменную при ручном вводе
        connect(le, &QLineEdit::textChanged, [&](const QString& text) { pathVar = text; });
        };

    if (algo == 0) { // A*
        createFileInput("Входные точки (.csv):", astarInput, "CSV Files (*.csv)", false);
        createFileInput("Файл результата (.txt):", astarOutput, "Text Files (*.txt)", true);
    }
    else if (algo == 1) { // TSP
        createFileInput("Точки (.csv):", tspInputCsv, "CSV Files (*.csv)", false);
        createFileInput("Файл результата (.txt):", tspOutput, "Text Files (*.txt)", true);
    }
    else if (algo == 2) { // VRP
        createFileInput("Транспорт (.csv):", vrpFleetCsv, "CSV Files (*.csv)", false);
        createFileInput("Заказы (.csv):", vrpOrdersCsv, "CSV Files (*.csv)", false);
        createFileInput("Файл результата (.txt):", vrpOutput, "Text Files (*.txt)", true);
    }

    QCheckBox* chkFull = new QCheckBox("Полный маршрут (с геометрией)");
    chkFull->setChecked(useFullOutput);
    layout.addRow("", chkFull);

    QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout.addRow(box);
    connect(box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() == QDialog::Accepted) {
        useFullOutput = chkFull->isChecked();
        logMessage("Настройки поиска сохранены.");
    }
}

void MainWindow::onSearchClicked() {
    if (graphBinPath.isEmpty() || !QFileInfo::exists(graphBinPath)) {
        QMessageBox::warning(this, "Ошибка", "Сначала необходимо загрузить карту!");
        return;
    }

    int algo = cbAlgorithm->currentIndex();

    // Проверяем заполненность путей
    if (algo == 0 && astarInput.isEmpty()) { QMessageBox::warning(this, "Ошибка", "Укажите входной файл для A* в настройках."); return; }
    if (algo == 1 && tspInputCsv.isEmpty()) { QMessageBox::warning(this, "Ошибка", "Укажите CSV с точками для TSP в настройках."); return; }
    if (algo == 2 && (vrpFleetCsv.isEmpty() || vrpOrdersCsv.isEmpty())) { QMessageBox::warning(this, "Ошибка", "Укажите CSV файлы для VRP в настройках."); return; }

    btnSearch->setEnabled(false);
    logMessage("Начинаем поиск маршрута...");

    // Асинхронный запуск поиска
    QFuture<bool> future = QtConcurrent::run([=]() -> bool {
        std::string err;

        if (algo == 0) { // A*
            std::string tempTxtPath = "temp_astar_input.txt";
            std::string parseError;

            // 1. Читаем пользовательский CSV через нашу функцию из csv_reader.cpp
            if (!parse_coordinate_pairs_csv_to_txt(astarInput.toStdString(), tempTxtPath, parseError)) {
                // Возвращаем false в QtConcurrent, если парсинг не удался
                return false;
            }

            // 2. Конвертируем координаты в плотные ID графа
            std::string queriesPath = "temp_queries.txt";
            if (!operations_research::convert_coordinates_to_ids(
                mapCsvPath.toStdString(), tempTxtPath, queriesPath, err)) {
                return false;
            }

            // 3. Запуск основного алгоритма A*
            FILE* gf = std::fopen(graphBinPath.toStdString().c_str(), "rb");
            FILE* inf = std::fopen(queriesPath.c_str(), "r");
            FILE* outf = std::fopen(astarOutput.toStdString().c_str(), "w");

            if (!gf || !inf || !outf) return false;

            run_search(gf, inf, outf, useFullOutput, currentMetric);

            std::fclose(gf);
            std::fclose(inf);
            std::fclose(outf);

            // 4. Подчищаем за собой временные файлы (используем стандартный C)
            std::remove(tempTxtPath.c_str());
            std::remove(queriesPath.c_str());

            return true;
        }
        else if (algo == 1) { // TSP
            // Поскольку TSP ожидает TXT файл с координатами под капотом, создаем его из CSV (как в main.cpp)
            std::vector<TspPoint> points;
            if (!load_tsp_points_from_csv(tspInputCsv.toStdString(), points, err)) return false;

            std::string tempTxt = "temp_tsp_input.txt";
            write_tsp_input_file(tempTxt, points);

            bool success = solve_tsp(tempTxt, graphBinPath.toStdString(), tspOutput.toStdString(),
                mapCsvPath.toStdString(), useFullOutput, currentMetric, err);
            std::remove(tempTxt.c_str());
            return success;

        }
        else if (algo == 2) { // VRP
            return solve_new_vrp(vrpOrdersCsv.toStdString(), vrpFleetCsv.toStdString(),
                graphBinPath.toStdString(), mapCsvPath.toStdString(),
                vrpOutput.toStdString(), currentMetric, useFullOutput, err);
        }
        return false;
        });

    searchWatcher.setFuture(future);
}

void MainWindow::onSearchFinished() {
    btnSearch->setEnabled(true);

    if (searchWatcher.result()) {
        logMessage("Поиск успешно завершен! Отображаем маршрут...");

        int algo = cbAlgorithm->currentIndex();
        QString currentOutput = (algo == 0) ? astarOutput : (algo == 1) ? tspOutput : vrpOutput;

        // Автоматически подсасываем результат для визуализатора
        viz.clear_routes();
        std::vector<Route> loaded;

        load_routes_from_file(currentOutput.toStdString(), loaded);

        for (const auto& r : loaded) {
            viz.add_route(r);
        }

        // Принудительно обновляем карту
        viz.updateVisuals();
        logMessage(QString("Загружено маршрутов: %1").arg(loaded.size()));

    }
    else {
        logMessage("Ошибка при поиске маршрута. Проверьте входные данные.");
    }
}