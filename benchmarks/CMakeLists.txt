# remove warning - lto is enabled in benchmark lib itself
unset(CMAKE_INTERPROCEDURAL_OPTIMIZATION)
set(CMAKE_CXX_STANDARD 17)

CPMAddPackage(
        NAME benchmark
        GITHUB_REPOSITORY google/benchmark
        VERSION 1.7.1
        OPTIONS
        "BENCHMARK_ENABLE_TESTING Off"
        "BENCHMARK_ENABLE_LTO ON"
)

file(GLOB_RECURSE BENCHMARKS_SOURCES "${PROJECT_SOURCE_DIR}/benchmarks/*.cpp")
add_executable(benchmarks ${BENCHMARKS_SOURCES})
target_include_directories(benchmarks PRIVATE ${PROJECT_SOURCE_DIR}/include ${benchmark_SOURCE_DIR})
target_link_libraries(benchmarks PRIVATE benchmark)