cmake_minimum_required(VERSION 3.20)
project(practica LANGUAGES CXX)
set(VCPKG_BUILD_TYPE release CACHE STRING "")

if (MSVC)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

option(RMVP_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

# OR-Tools is optional. Default AUTO means:
# - if an extracted OR-Tools C++ binary distribution is found, TSP support is enabled;
# - if it is not found, the project still builds without OR-Tools/TSP.
set(WITH_ORTOOLS_TSP "AUTO" CACHE STRING "OR-Tools TSP support: AUTO, ON, or OFF")
set_property(CACHE WITH_ORTOOLS_TSP PROPERTY STRINGS AUTO ON OFF)
option(WITH_ORTOOLS_FETCHCONTENT "Download and build OR-Tools from source with FetchContent" ON)
set(ORTOOLS_ROOT "" CACHE PATH "Optional path to extracted OR-Tools C++ binary distribution")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ---------------- Optional Google OR-Tools auto-detection ----------------
set(EASY_WAY_ORTOOLS_ENABLED OFF)

string(TOUPPER "${WITH_ORTOOLS_TSP}" WITH_ORTOOLS_TSP_UPPER)
if (NOT WITH_ORTOOLS_TSP_UPPER STREQUAL "AUTO" AND
    NOT WITH_ORTOOLS_TSP_UPPER STREQUAL "ON" AND
    NOT WITH_ORTOOLS_TSP_UPPER STREQUAL "OFF")
    message(FATAL_ERROR "WITH_ORTOOLS_TSP must be AUTO, ON, or OFF")
endif()

if (WITH_ORTOOLS_FETCHCONTENT AND WITH_ORTOOLS_TSP_UPPER STREQUAL "OFF")
    message(FATAL_ERROR "WITH_ORTOOLS_FETCHCONTENT=ON conflicts with WITH_ORTOOLS_TSP=OFF")
endif()

if (WITH_ORTOOLS_FETCHCONTENT)
    include(FetchContent)

    set(BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

    set(BUILD_HiGHS OFF CACHE BOOL "" FORCE)

    set(USE_HIGHS OFF CACHE BOOL "" FORCE)
    set(BUILD_XPRESS OFF CACHE BOOL "" FORCE)
    set(BUILD_GUROBI OFF CACHE BOOL "" FORCE)

    # TSP/VRP does not need these external MIP solvers.
    set(USE_SCIP OFF CACHE BOOL "" FORCE)
    set(USE_COINOR OFF CACHE BOOL "" FORCE)

    set(BUILD_CSHARP OFF CACHE BOOL "" FORCE)
    set(BUILD_JAVA OFF CACHE BOOL "" FORCE)
    set(BUILD_PYTHON OFF CACHE BOOL "" FORCE)
    set(BUILD_DOTNET OFF CACHE BOOL "" FORCE)

    #set(BUILD_DEPS ON CACHE INTERNAL "")
    set(BUILD_DEPS ON CACHE BOOL "" FORCE)
    set(BUILD_protobuf ON CACHE BOOL "" FORCE)
    set(BUILD_abseil-cpp ON CACHE BOOL "" FORCE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /MP")

    set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)
    list(APPEND CMAKE_PROGRAM_PATH "${CMAKE_BINARY_DIR}/_deps/protobuf-build")
    

    FetchContent_Declare(
        ortools
        GIT_REPOSITORY https://github.com/google/or-tools.git
        GIT_TAG        v9.11
        GIT_SHALLOW    TRUE
    )

    FetchContent_MakeAvailable(ortools)
    set(EASY_WAY_ORTOOLS_ENABLED ON)
    message(STATUS "OR-Tools: enabled via FetchContent")

elseif (NOT WITH_ORTOOLS_TSP_UPPER STREQUAL "OFF")
    # 1.                                     
    if (NOT ORTOOLS_ROOT)
        set(ORTOOLS_ROOT "${PROJECT_SOURCE_DIR}/external/or-tools")
    endif()

    # 2.                                      CMake
    #              find_package                    external/or-tools                    
    list(APPEND CMAKE_PREFIX_PATH "${ORTOOLS_ROOT}")
    
    find_package(ortools CONFIG QUIET)

    if (ortools_FOUND)
        set(EASY_WAY_ORTOOLS_ENABLED ON)
        set(EASY_WAY_ORTOOLS_ROOT "${ORTOOLS_ROOT}")
        message(STATUS "OR-Tools: found via Config package at ${EASY_WAY_ORTOOLS_ROOT}")
    elseif (WITH_ORTOOLS_TSP_UPPER STREQUAL "ON")
        message(FATAL_ERROR
            "WITH_ORTOOLS_TSP=ON, but OR-Tools was not found.\n"
            "Please check that '${ORTOOLS_ROOT}' contains valid OR-Tools C++ binaries."
        )
    else()
        message(STATUS "OR-Tools: not found, building without TSP/VRP support")
    endif()
else()
    message(STATUS "OR-Tools: disabled by WITH_ORTOOLS_TSP=OFF")
endif()

# ---------------- Project sources ----------------
set(GRAPH_BUILDER_SOURCES
    main.cpp
    src/osm_graph.cpp
    src/preprocess.cpp
    src/search.cpp
    src/geocode.cpp
    src/visualizer.cpp
    src/graph_loader.cpp 
    src/vrp_solver.cpp
    src/csv_reader.cpp
)

# Optional local TSP implementation file.
if (TARGET ortools::ortools OR EASY_WAY_ORTOOLS_ENABLED)
    set(ORTOOLS_FOUND_AND_READY ON)
else()
    set(ORTOOLS_FOUND_AND_READY OFF)
endif()

#      OR-Tools               ,                           TSP
if (ORTOOLS_FOUND_AND_READY)
    list(APPEND GRAPH_BUILDER_SOURCES src/tsp.cpp)
endif()

add_executable(graph_builder ${GRAPH_BUILDER_SOURCES})

# 1. Ищем пакет OpenMP в системе
find_package(OpenMP REQUIRED)

find_package(Boost REQUIRED)

# 2. Если пакет найден, связываем его с вашим исполняемым файлом
if(OpenMP_CXX_FOUND)
    target_link_libraries(graph_builder PRIVATE OpenMP::OpenMP_CXX)
endif()

if(Boost_FOUND)
    target_include_directories(graph_builder PRIVATE ${Boost_INCLUDE_DIRS})
endif()

target_compile_features(graph_builder PRIVATE cxx_std_20)

set_target_properties(graph_builder PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# --- deps from vcpkg ---
find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

# libosmium is header-only, but it needs dependencies for reading pbf.
find_path(OSMIUM_INCLUDE_DIR osmium/version.hpp REQUIRED)

find_package(ZLIB REQUIRED)
find_package(BZip2 REQUIRED)
find_package(expat REQUIRED)

target_include_directories(graph_builder PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${OSMIUM_INCLUDE_DIR}"
)

if (ORTOOLS_FOUND_AND_READY)
    target_include_directories(graph_builder BEFORE PRIVATE 
        "${CMAKE_BINARY_DIR}/_deps/absl-src"
        "${CMAKE_BINARY_DIR}/_deps/ortools-src"
    )

    target_compile_definitions(graph_builder PRIVATE 
        ABSL_CONSUME_DLL
    )


    target_compile_definitions(graph_builder PRIVATE WITH_ORTOOLS_TSP=1)
    target_link_libraries(graph_builder 
    PRIVATE 
        ortools::ortools
        
)

    if (WIN32 AND NOT WITH_ORTOOLS_FETCHCONTENT)
        file(GLOB ORTOOLS_RUNTIME_DLLS CONFIGURE_DEPENDS
            "${EASY_WAY_ORTOOLS_ROOT}/bin/*.dll"
            "${EASY_WAY_ORTOOLS_ROOT}/lib/*.dll"
        )
        if (ORTOOLS_RUNTIME_DLLS)
            add_custom_command(TARGET graph_builder POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${ORTOOLS_RUNTIME_DLLS}
                    "$<TARGET_FILE_DIR:graph_builder>"
                COMMENT "Copying OR-Tools runtime DLLs"
                VERBATIM
            )
        endif()
    endif()
endif()


target_link_libraries(graph_builder PRIVATE
    Qt6::Widgets
    nlohmann_json::nlohmann_json
    spdlog::spdlog
    ZLIB::ZLIB
    BZip2::BZip2
    expat::expat
)

# ---------------- Windows & Compiler Flags ----------------
if (WIN32)
    # Critical for libosmium on Windows: min/max can break std::numeric_limits::max.
    target_compile_definitions(graph_builder PRIVATE
        NOMINMAX
        WIN32_LEAN_AND_MEAN
        VC_EXTRALEAN
        _CRT_SECURE_NO_WARNINGS
    )

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

if (MSVC)
    target_compile_options(graph_builder PRIVATE
        /W4
        /permissive-
        /utf-8
        /bigobj
        /Zc:__cplusplus  #           :                          ,               __cplusplus                        C++20
    )
    
    #           :                                    "/FORCE:MULTIPLE",                                        .

    if (WITH_ORTOOLS_FETCHCONTENT)
        target_compile_options(graph_builder PRIVATE /Zm2000)
    endif()

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