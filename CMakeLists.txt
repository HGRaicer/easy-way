cmake_minimum_required(VERSION 3.20)
project(practica LANGUAGES CXX)

option(RMVP_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_executable(graph_builder
    main.cpp
    src/osm_graph.cpp
    src/preprocess.cpp
    src/search.cpp
    src/visualizer.cpp
    src/graph_loader.cpp
)

target_compile_features(graph_builder PRIVATE cxx_std_17)

set_target_properties(graph_builder PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# --- deps from vcpkg ---
find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

# libosmium: header-only, but it needs dependencies for reading pbf
find_path(OSMIUM_INCLUDE_DIR osmium/version.hpp REQUIRED)

find_package(ZLIB REQUIRED)
find_package(BZip2 REQUIRED)
find_package(EXPAT REQUIRED)

target_include_directories(graph_builder PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${OSMIUM_INCLUDE_DIR}"
)

target_link_libraries(graph_builder PRIVATE
    Qt6::Widgets
    nlohmann_json::nlohmann_json
    spdlog::spdlog
    ZLIB::ZLIB
    BZip2::BZip2
    EXPAT::EXPAT
)

if (WIN32)
    set(WINDEPLOYQT_EXE
        "${CMAKE_BINARY_DIR}/vcpkg_installed/${VCPKG_TARGET_TRIPLET}/tools/Qt6/bin/windeployqt.exe"
    )

    if (EXISTS "${WINDEPLOYQT_EXE}")
        add_custom_command(TARGET graph_builder POST_BUILD
            COMMAND "${WINDEPLOYQT_EXE}" "$<TARGET_FILE:graph_builder>"
            COMMENT "Deploying Qt runtime files with windeployqt"
            VERBATIM
        )
    else()
        message(WARNING "windeployqt.exe not found: ${WINDEPLOYQT_EXE}")
    endif()
endif()

if (WIN32)
    # critical for libosmium on Windows: min/max can break std::numeric_limits::max
    target_compile_definitions(graph_builder PRIVATE
        NOMINMAX
        WIN32_LEAN_AND_MEAN
        VC_EXTRALEAN
        _CRT_SECURE_NO_WARNINGS
    )
endif()

if (MSVC)
    target_compile_options(graph_builder PRIVATE /W4 /permissive- /Zc:__cplusplus /utf-8)
    if (RMVP_WARNINGS_AS_ERRORS)
        target_compile_options(graph_builder PRIVATE /WX)
    endif()
else()
    target_compile_options(graph_builder PRIVATE -Wall -Wextra -Wpedantic)
    if (RMVP_WARNINGS_AS_ERRORS)
        target_compile_options(graph_builder PRIVATE -Werror)
    endif()
endif()

if (COMMAND qt_finalize_executable)
    qt_finalize_executable(graph_builder)
endif()
