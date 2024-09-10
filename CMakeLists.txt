cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)

project(kdsp-cpp)

# https://github.com/cpm-cmake/CPM.cmake/releases/tag/v0.38.1
include(cmake/CPM.cmake)

# add dependencies
CPMAddPackage("gh:fmtlib/fmt#6.1.2")
CPMAddPackage("gh:gabime/spdlog@1.6.1")
CPMAddPackage("gh:jarro2783/cxxopts@3.1.1")

# enable flto for non-debug builds
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_DEBUG FALSE)

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCES})

if (MSVC)
    # -march=native equivalent for MSVC?
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Release>:/O2 /DNDEBUG>)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:/Od >)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:RelWithDebInfo>:/O2>)
else ()
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Release>:-O3 -flto -frename-registers -march=native -DNDEBUG -s>)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:-O0 -ggdb -march=native>)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:RelWithDebInfo>:-O3 -flto -frename-registers -ggdb -march=native>)

    set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-flto")
endif ()

target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} PRIVATE fmt::fmt)
target_link_libraries(${PROJECT_NAME} PRIVATE spdlog)
target_link_libraries(${PROJECT_NAME} PRIVATE cxxopts)

if (MSVC)
else ()
    target_link_libraries(${PROJECT_NAME} PRIVATE stdc++fs)
endif ()

add_subdirectory(catch2_tests)

add_subdirectory(benchmarks)
