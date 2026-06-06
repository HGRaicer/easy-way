# easy-way

## Установка зависимостей (vcpkg)

Проект использует **vcpkg manifest mode** (файл `vcpkg.json` в корне).  
Зависимости будут установлены автоматически при конфигурации CMake, если подключён vcpkg toolchain.

### Windows (MSVC + Visual Studio)

1) Установить Visual Studio 2022 с workload **Desktop development with C++**.

2) Установить vcpkg (в любое место, например `C:\dev\vcpkg`):
```powershell
git clone https://github.com/microsoft/vcpkg.git C:\dev\vcpkg
C:\dev\vcpkg\bootstrap-vcpkg.bat
```

3) Указать путь к vcpkg.

в `CMakePresets.json`  
Найти строку:
```json
"CMAKE_TOOLCHAIN_FILE": "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
```
и заменить на свой путь, например:
```json
"CMAKE_TOOLCHAIN_FILE": "C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake"
```

4) Сборка (Debug/Release):
```powershell
cmake --preset msvc-debug
cmake --build --preset msvc-debug
```

Release (с оптимизациями компилятора):
```powershell
cmake --preset msvc-release
cmake --build --preset msvc-release
```

Где будет exe:
- `out/build/msvc-debug/bin/graph_builder.exe`
- `out/build/msvc-release/bin/graph_builder.exe`

---

### Linux (GCC/Clang)

1) Установить инструменты сборки (пример для Ubuntu/Debian):
```bash
sudo apt update
sudo apt install -y build-essential git cmake ninja-build pkg-config \
                   libx11-dev libxft-dev libxext-dev libgl1-mesa-dev \
                   libglu1-mesa-dev libxrandr-dev libxi-dev libxcursor-dev \
                   libxkbcommon-dev libxkbcommon-x11-dev libfontconfig1-dev
```

2) Установить vcpkg:
```bash
git clone https://github.com/microsoft/vcpkg.git ~/vcpkg
~/vcpkg/bootstrap-vcpkg.sh
```

3) Указать `VCPKG_ROOT`:
```bash
export VCPKG_ROOT=~/vcpkg
```
(лучше добавить в `~/.bashrc` / `~/.zshrc`)

4) Сборка без пресетов:
```bash
cmake -S . -B out/build/linux-release -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" \
  -DVCPKG_TARGET_TRIPLET=x64-linux

cmake --build out/build/linux-release
```

---

## Запуск (вручную)

### 0. Данные
Положить файл карты в папку `data/`, например:
- `data/your_city.osm.pbf`

---

### 1. Проверка чтения pbf
**Windows:**
```powershell
out\build\msvc-release\bin\graph_builder.exe osm-info --pbf data\your_city.osm.pbf
```

**Linux:**
```bash
./out/build/linux-release/bin/graph_builder osm-info --pbf data/your_city.osm.pbf
```

---

### 2. Preprocess → graph.bin + id_map.csv
Создаёт:
- `graph.bin` — граф для поиска
- `id_map.csv` — соответствие `dense_id ↔ osm_id`

**Windows:**
```powershell
out\build\msvc-release\bin\graph_builder.exe preprocess --pbf data\your_city.osm.pbf --out graph.bin --map id_map.csv
```

**Linux:**
```bash
./out/build/linux-release/bin/graph_builder preprocess --pbf data/your_city.osm.pbf --out graph.bin --map id_map.csv
```

---

### 3. Search (A*) по запросам из input.csv
создать в корне проекта файл `input.csv` — по одной паре на строку:
```
<source_lat> <source_long> <target_lat> <target_long>
```
где `<source_lat>` `<source_long>` `<target_lat>` `<target_long>` — Широта и долгота начала и конца маршрута.

Пример `input.csv`:
```
lat,lon,lat2,lon2
52.595,38.408,57.2603,32.7003
55.555,35.555,56.666,34.444
```
По времени:

**Windows:**
```powershell
out\build\msvc-release\bin\graph_builder.exe search --graph graph.bin --in input.txt --out result.txt --metric time --full
```

**Linux:**
```bash
./out/build/linux-release/bin/graph_builder search --graph graph.bin --in input.txt --out result.txt --metric time --full
```

Формат `result.txt`:
- без `--full`: `time distance` или `-1`
- с `--full`: `time distance k v1 v2 ... vk`

По расстоянию:

**Windows:**

```powershell
out\build\msvc-release\bin\graph_builder.exe search --graph graph.bin --in input.txt --out result.txt --metric distance --full
```

**Linux:**

```bash
./out/build/linux-release/bin/graph_builder search --graph graph.bin --in input.txt --out result.txt --metric distance --full
```


---

### 5. Visualize

Отображение маршрута из файла результатов.

**Windows:**

```powershell
out\build\msvc-release\bin\graph_builder.exe visualize --graph graph.bin --result result.txt
```

**Linux:**

```bash
./out/build/linux-release/bin/graph_builder visualize --graph graph.bin --result result.txt
```

---

### 6. TSP (Travelling Salesman Problem)

Подготовить файл `points.csv` формата
```
id, lat,lon
1, 55.7558, 37.6173
2, 55.5714, 37.6856
3, 55.8094, 37.4536
4, 55.6944, 37.3483
```

По расстоянию:

**Windows**

```powershell
out\build\msvc-release\bin\graph_builder.exe tsp --graph graph.bin --csv points.csv --out tsp_result.txt --map id_map.csv --metric distance --full
```

**Linux**

```bash
./out/build/linux-release/bin/graph_builder tsp --graph graph.bin --csv points.csv --out tsp_result.txt --map id_map.csv --metric distance --full
```

По времени:

```bash
tsp --graph graph.bin --csv points.csv --out tsp_result.txt --map id_map.csv --metric time --full
```

---

### 7. VRP (Vehicle Routing Problem)

Подготовить файлы:

* `vrp_points.csv` формата
```
id,lat,lon,demand,tw_start,tw_end
0,55.755826,37.617299,0,0,86400
1,55.751244,37.618421,15,0,86400
2,55.759322,37.625103,40,0,86400
3,55.741511,37.612033,25,10000,50400
4,55.762111,37.601122,30,0,86400
5,55.733222,37.634455,20,70000,86400
```
* `fleet.csv` формата
```
vehicle_id,capacity
1,50
2,100
3,45
```

По расстоянию:

```bash
vrp --graph graph.bin --csv vrp_points.csv --out vrp_result.txt --map id_map.csv --metric distance --fleet fleet.csv --full
```

По времени:

```bash
vrp --graph graph.bin --csv vrp_points.csv --out vrp_result.txt --map id_map.csv --metric time --fleet fleet.csv --full
```


-----------------

## Запуск (автоматизированный для MSVC)

### 0 Данные и входные параметры
Положить файл карты в папку `data/`, например:
- `data/your_city.osm.pbf`

### Для А*

создать в корне проекта файл `input.csv` — по одной паре на строку:
```
<source_lat> <source_long> <target_lat> <target_long>
```
где `<source_lat>` `<source_long>` `<target_lat>` `<target_long>` — Широта и долгота начала и конца маршрута.

Пример `input.csv`:
```
lat,lon,lat2,lon2
52.595,38.408,57.2603,32.7003
55.555,35.555,56.666,34.444
```
### Для TSP

Подготовить файл `points.csv` формата
```
id, lat,lon
1, 55.7558, 37.6173
2, 55.5714, 37.6856
3, 55.8094, 37.4536
4, 55.6944, 37.3483
```

### Для VRP

Подготовить файлы:

* `vrp_points.csv` формата
```
id,lat,lon,demand,tw_start,tw_end
0,55.755826,37.617299,0,0,86400
1,55.751244,37.618421,15,0,86400
2,55.759322,37.625103,40,0,86400
3,55.741511,37.612033,25,10000,50400
4,55.762111,37.601122,30,0,86400
5,55.733222,37.634455,20,70000,86400
```
* `fleet.csv` формата
```
vehicle_id,capacity
1,50
2,100
3,45
```
---

### 1. Создание пресетов

В скрытой папке `.vs` создать файл `launch.vs.json` и заменить your_city.osm.pbf на файл исходной карты:
```json
{
  "version": "0.2.1",
  "configurations": [
    {
      "name": "OSM info",
      "type": "default",
      "project": "CMakeLists.txt",
      "projectTarget": "",
      "args": [
        "osm-info",
        "--pbf",
        "../../data/your_city.osm.pbf"
      ],
      "cwd": "${workspaceRoot}"
    },
    {
      "name": "Preprocess",
      "type": "default",
      "project": "CMakeLists.txt",
      "projectTarget": "",
      "args": [
        "preprocess",
        "--pbf",
        "../../data/your_city.osm.pbf",
        "--out",
        "graph.bin",
        "--map",
        "id_map.csv"
      ],
      "cwd": "${workspaceRoot}"
    },
    {
      "name": "Search time",
      "type": "default",
      "project": "CMakeLists.txt",
      "projectTarget": "",
      "args": [
        "search",
        "--graph",
        "graph.bin",
        "--in",
        "../../input.txt",
        "--out",
        "../../../result.txt",
        "--full",
        "--metric",
        "time"
      ],
      "cwd": "${workspaceRoot}"
    },
    {
      "name": "Search dist",
      "type": "default",
      "project": "CMakeLists.txt",
      "projectTarget": "",
      "args": [
        "search",
        "--graph",
        "graph.bin",
        "--in",
        "../../input.txt",
        "--out",
        "../../../result.txt",
        "--full",
        "--metric",
        "distance"
      ],
      "cwd": "${workspaceRoot}"
    },
    {
      "name": "Visualize",
      "type": "default",
      "project": "CMakeLists.txt",
      "projectTarget": "",
      "args": [
        "visualize",
        "--graph",
        "graph.bin",
        "--result",
        "../../../result.txt"
      ],
      "cwd": "${workspaceRoot}"
    },
    {
      "name": "TSP (distance)",
      "type": "default",
      "project": "CMakeLists.txt",
      "projectTarget": "",
      "args": [
        "tsp",
        "--graph",
        "graph.bin",
        "--csv",
        "../../points.csv",
        "--out",
        "../../../tsp_result1.txt",
        "--map",
        "id_map.csv",
        "--metric",
        "distance",
        "--full"
      ],
      "cwd": "${workspaceRoot}"
    },
    {
      "name": "TSP (time)",
      "type": "default",
      "project": "CMakeLists.txt",
      "projectTarget": "",
      "args": [
        "tsp",
        "--graph",
        "graph.bin",
        "--csv",
        "../../points.csv",
        "--out",
        "../../../tsp_result.txt",
        "--map",
        "id_map.csv",
        "--metric",
        "time",
        "--full"
      ],
      "cwd": "${workspaceRoot}"
    },
    {
      "name": "VRP (time)",
      "type": "default",
      "project": "CMakeLists.txt",
      "projectTarget": "",
      "args": [
        "vrp",
        "--graph",
        "graph.bin",
        "--csv",
        "../../vrp_points.csv",
        "--out",
        "../../../vrp_result.txt",
        "--map",
        "id_map.csv",
        "--metric",
        "time",
        "--fleet",
        "../../fleet.csv",
        "--full"
      ],
      "cwd": "${workspaceRoot}"
    },
    {
      "name": "VRP (distance)",
      "type": "default",
      "project": "CMakeLists.txt",
      "projectTarget": "",
      "args": [
        "vrp",
        "--graph",
        "graph.bin",
        "--csv",
        "../../vrp_points.csv",
        "--out",
        "../../../vrp_result.txt",
        "--map",
        "id_map.csv",
        "--metric",
        "distance",
        "--fleet",
        "../../fleet.csv",
        "--full"
      ],
      "cwd": "${workspaceRoot}"
    },
    {
      "type": "default",
      "project": "CMakeLists.txt",
      "projectTarget": "",
      "name": "CMakeLists.txt"
    }
  ]
}
```
