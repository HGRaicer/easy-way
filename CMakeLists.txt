cmake_minimum_required(VERSION 3.20)
project(practica LANGUAGES CXX)

option(RMVP_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

add_executable(graph_builder
    main.cpp
    src/osm_graph.cpp
    src/preprocess.cpp
    src/search.cpp
)

target_compile_features(graph_builder PRIVATE cxx_std_17)

set_target_properties(graph_builder PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# --- deps from vcpkg ---
find_package(nlohmann_json CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

# libosmium: header-only (но тянет зависимости для чтения)
find_path(OSMIUM_INCLUDE_DIR osmium/version.hpp REQUIRED)

find_package(ZLIB REQUIRED)
find_package(BZip2 REQUIRED)
find_package(EXPAT REQUIRED)

target_include_directories(graph_builder PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${OSMIUM_INCLUDE_DIR}"
)

target_link_libraries(graph_builder PRIVATE
    nlohmann_json::nlohmann_json
    spdlog::spdlog
    ZLIB::ZLIB
    BZip2::BZip2
    EXPAT::EXPAT
)

if (WIN32)
    # критично для libosmium на Windows (min/max ломают std::numeric_limits::max)
    target_compile_definitions(graph_builder PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
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
