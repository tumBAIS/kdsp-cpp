#include <iostream>
#include <fstream>
#include <string>
#ifndef _MSC_VER
#include <sys/time.h>
#endif
#include <tuple>
#include <chrono>
#include <bitset>
#include <unordered_map>
#include <sstream>
#include <kdisjoint.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/cfg/env.h>
#include <cxxopts.hpp>


template<class Vertex, class Weight>
auto arc_string_to_tuple(std::string &string) -> std::tuple<Vertex, Vertex, Weight> {
    std::stringstream stringstream(string);
    std::string part;

    std::getline(stringstream,part,',');
    auto u = stoi(part);

    std::getline(stringstream,part,',');
    auto v = stoi(part);

    std::getline(stringstream,part,',');
    auto w = stoi(part);

    return std::make_tuple(u, v, w);
}


template<class Vertex, class Weight>
auto load(std::string instance_path) -> CompactDiGraph<Vertex, Weight> {
    std::ifstream infile(instance_path);
    std::string line;
    // num_nodes
    std::getline(infile, line, '\n');
    auto num_nodes = (Vertex) stoi(line);
    // num_arcs
    std::getline(infile, line);
    auto num_arcs = stoi(line);
    // empty line
    std::getline(infile, line);

    std::vector<std::tuple<Vertex, Vertex, Weight>> arc_tuples;

    while (std::getline(infile, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        auto tuple = arc_string_to_tuple<Vertex, Weight>(line);
        arc_tuples.push_back(tuple);
    }

    return CompactDiGraph(num_nodes, arc_tuples);
}

int main(int argc, char* argv[]) {
    spdlog::cfg::load_env_levels();
    cxxopts::Options options("kdsp-cpp", "A k-dSP problem solver written in C++");
    options.add_options()
            ("i,instance", "Instance path", cxxopts::value<std::string>())
            ("h,help", "Print usage");

    auto result = options.parse(argc, argv);
    if (result.count("help") || !result.count("instance"))
    {
      std::cout << options.help() << std::endl;
      exit(0);
    }

    auto instance_path = result["instance"].as<std::string>();

    spdlog::info("reading instance from {}", instance_path);
    auto graph = load<unsigned int, int>(instance_path);
    spdlog::info("start k-disjoint shortest path");
    auto t_start = std::chrono::steady_clock::now();
    auto [k, sp_arcs] = kdisjoint_shortest_path<unsigned int, int>(graph, 0, 1);
    auto t_end = std::chrono::steady_clock::now();
    auto t_dur = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
    spdlog::info("finished k-disjoint shortest path after {}ms", t_dur);

    // graph weights may have been modified -> reload instance
    graph = load<unsigned int, int>(instance_path);
    auto sum = 0;
    for(auto [u, v] : sp_arcs) {
        sum += graph.get_edge(u, v)->w;
    }
    std::cout << k << "," << sum << "," << t_dur << std::endl;

    return 0;
}
