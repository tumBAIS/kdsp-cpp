#ifdef _WIN32
#pragma comment ( lib, "Shlwapi.lib" )
#ifdef _DEBUG
#pragma comment ( lib, "benchmarkd.lib" )
#else
#pragma comment ( lib, "benchmark.lib" )
#endif
#endif

#include <benchmark/benchmark.h>

#include <algorithm>
#include <limits>
#include <vector>
#include <kdisjoint.hpp>
#include <random>
#include <tuple>

using Vertex = std::size_t;
using Weight = int;
using Edge = std::tuple<Vertex, Vertex, Weight>;

static auto create_instance(int width, int length, std::pair<int, int> weight, std::pair<int, int> links) ->
CompactDiGraph<Vertex , Weight>;

// helper to handle defaults
template <int N, class E, class ...T>
constexpr auto tuple_get_or_default(std::tuple<T...> tuple, E default_value) -> E {
    if constexpr (std::tuple_size<std::tuple<T...>>::value > N) { return std::get<N>(tuple); } else { return default_value; }
}

template <class ...ExtraArgs>
static void run_kdisjoint_shortest_path(benchmark::State& state, ExtraArgs&&... extra_args) {
    static_assert(std::tuple_size<std::tuple<ExtraArgs...>>::value > 1);

    std::tuple<ExtraArgs...> params(extra_args...);
    auto width = std::get<0>(params);
    auto length = std::get<1>(params);
    auto weight = tuple_get_or_default<2>(params, std::pair<int, int>(-100, 100));
    auto links = tuple_get_or_default<3>(params, std::pair<int, int>(width, width));

    CompactDiGraph<Vertex, Weight> graph = create_instance(width, length, weight, links);
    for (auto _ : state) {
        benchmark::DoNotOptimize(kdisjoint_shortest_path<Vertex, Weight>(graph, 0, 1));
    }
}

BENCHMARK_CAPTURE(run_kdisjoint_shortest_path, W10x10, 10, 10);
BENCHMARK_CAPTURE(run_kdisjoint_shortest_path, W10x20, 10, 20);
BENCHMARK_CAPTURE(run_kdisjoint_shortest_path, W20x10, 20, 10);
BENCHMARK_CAPTURE(run_kdisjoint_shortest_path, W20x20, 20, 20);
BENCHMARK_CAPTURE(run_kdisjoint_shortest_path, W20x20, 20, 20);

BENCHMARK_CAPTURE(run_kdisjoint_shortest_path, W200x10, 200, 10);
BENCHMARK_CAPTURE(run_kdisjoint_shortest_path, W200x20, 200, 20);

BENCHMARK_CAPTURE(run_kdisjoint_shortest_path, W4x400, 4, 400);
BENCHMARK_CAPTURE(run_kdisjoint_shortest_path, W4x600, 4, 600);
BENCHMARK_CAPTURE(run_kdisjoint_shortest_path, W4x800, 4, 800);

static auto create_instance(int width, int length, std::pair<int, int> weight, std::pair<int, int> links) ->
CompactDiGraph<Vertex, Weight> {
    std::default_random_engine g(101);
    std::uniform_real_distribution<> real_dist(0.0, 1.0);
    auto next_int_lb_ub = [&](int lb, int ub){ return std::lround(real_dist(g) * static_cast<double>(ub - lb)) + lb; };
    auto next_int_ub = [&](int ub){ return std::lround(real_dist(g) * static_cast<double>(ub)); };

    const Vertex num_vertices = width * length + 2;

    const int min_links = std::min(links.first, width);
    const int max_links = std::min(links.second, width);
    auto next_num_links = [&](){ return next_int_lb_ub(min_links, max_links); };

    const Weight min_weight = links.first;
    const Weight max_weight = links.second;
    auto next_weight = [&](){ return next_int_lb_ub(min_weight, max_weight); };

    std::vector<Edge> edges;
    // edges from the sourc
    for (auto i = 0; i < width; ++i) {
        edges.emplace_back(static_cast<Vertex>(0), static_cast<Vertex>(i+1), 0);
    }

    std::vector<int> shuffled;
    shuffled.resize(width);
    for(auto i = 0; i < width; ++i) { shuffled[i] = i; }

    for (auto j = 0; j < length-1; ++j) {
        std::shuffle(shuffled.begin(), shuffled.end(), g);
        for (auto i = 0; i < width; ++i) {
            for (auto l = next_num_links()-1; l >= 0; --l) {
                edges.emplace_back(
                        static_cast<Vertex>(1 + width * j + i),
                        static_cast<Vertex>(1 + width * (j+1) + shuffled[l]),
                        next_weight());
            }
        }
    }
    // edges to the sink
    for (auto i = 0; i < width; ++i) {
        edges.emplace_back(
                static_cast<Vertex>(i + 1 + width*(length-1)),
                static_cast<Vertex>(num_vertices - 1),
                0);
    }

    return CompactDiGraph<Vertex, Weight>(num_vertices, edges);
}
