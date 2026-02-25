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
sudo apt install -y build-essential git cmake ninja-build pkg-config
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

### 3. Search (A*) по запросам из queries.txt
создать в корне проекта файл `queries.txt` — по одной паре на строку:
```
<source_id> <target_id>
```
где `source_id` и `target_id` — **dense id 1..N** (это НЕ OSM id).

Пример `queries.txt`:
```
1 10
10 20
```

**Windows:**
```powershell
out\build\msvc-release\bin\graph_builder.exe search --graph graph.bin --in queries.txt --out result.txt --full
```

**Linux:**
```bash
./out/build/linux-release/bin/graph_builder search --graph graph.bin --in queries.txt --out result.txt --full
```

Формат `result.txt`:
- без `--full`: `distance` или `-1`
- с `--full`: `distance k v1 v2 ... vk`

-----------------

## Запуск (автоматизированный для MSVC)

### 0 Данные и входные параметры
Положить файл карты в папку `data/`, например:
- `data/your_city.osm.pbf`

Создать в корне проекта файл `queries.txt` — по одной паре на строку:
```
<source_id> <target_id>
```
где `source_id` и `target_id` — **dense id 1..N** (это НЕ OSM id).

Пример `queries.txt`:
```
1 10
10 20
```
---

### 1. Создание пресетов

В скрытой папке `.vs` создать файл `launch.vs.json`
**launch.vs.json:**
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
      "name": "Search",
      "type": "default",
      "project": "CMakeLists.txt",
      "projectTarget": "",
      "args": [
        "search",
        "--graph",
        "graph.bin",
        "--in",
        "../../queries.txt",
        "--out",
        "../../../result.txt",
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
