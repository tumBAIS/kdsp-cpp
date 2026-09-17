#include <cmath>
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

#include <lemon/static_graph.h>
#include <lemon/cost_scaling.h>
#include <lemon/network_simplex.h>


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

template<typename Vertex, typename Weight>
auto lemon_costscaling_k_disjoint_shortest_path(CompactDiGraph<Vertex, Weight> &graph, Vertex source,
                                                   Vertex target) -> std::pair<int, std::vector<std::pair<Vertex, Vertex>>> {

    typedef std::pair<Vertex, Vertex> Edge;
    std::vector<Edge> arcs;
    auto edge_it = graph.edges_begin();
    for (auto i = 0; edge_it != graph.edges_end(); edge_it++, i++) {
        auto from = edge_it->from;
        auto to = edge_it->to == target ? edge_it->to : (graph.num_nodes() - 2) + edge_it->to;
        auto arc = std::make_pair(from, to);
        arcs.push_back(arc);
    }
    auto num_arcs = arcs.size();

    for (auto j = 0; j < graph.num_nodes() - 2; j++) {
        auto i = num_arcs + j;
        auto from = (graph.num_nodes() - 2) + (j + 2);
        auto to = (j + 2);
        auto arc = std::make_pair(from, to);
        arcs.push_back(arc);
    }

    lemon::StaticDigraph sg;
    sg.build(graph.num_nodes() * 2 - 2, arcs.begin(), arcs.end());
    lemon::StaticDigraph::ArcMap<Weight> costMap(sg);
    lemon::StaticDigraph::ArcMap<int> upperMap(sg);

    // std::cout << "using lemon" << std::endl;

    edge_it = graph.edges_begin();
    for (auto i = 0; edge_it != graph.edges_end(); edge_it++, i++) {
        upperMap[sg.arc(i)] = 1;
        costMap[sg.arc(i)] = edge_it->w;
    }

    for (auto j = 0; j < graph.num_nodes() - 2; j++) {
        auto i = num_arcs + j;
        upperMap[sg.arc(i)] = 1;
        costMap[sg.arc(i)] = 0;
    }
    lemon::CostScaling<lemon::StaticDigraph> cs(sg);


    auto [e_it, e_end] = graph.out_edges(source);
    cs.upperMap(upperMap).costMap(costMap).stSupply(sg.node(source), sg.node(target), std::distance(e_it, e_end)).run();
//    cs.upperMap(upperMap).costMap(costMap).stSupply(sg.node(source), sg.node(target), 5).run();

    lemon::StaticDigraph::ArcMap<int> flowMap(sg);
    cs.flowMap(flowMap);

    std::vector<Edge> sp_edges;
    edge_it = graph.edges_begin();
    auto k = 0;
    for (auto i = 0; edge_it != graph.edges_end(); edge_it++, i++) {
        auto from = edge_it->from;
        auto to = edge_it->to == target ? edge_it->to : (graph.num_nodes() - 2) + edge_it->to;
        auto flow = flowMap[sg.arc(i)];
        if (flow > 0) {
            sp_edges.emplace_back(edge_it->from, edge_it->to);
            if (from == 0) {
                k += 1;
            }
        }
    }

    return std::make_pair(k, sp_edges);
}

template<typename Vertex, typename Weight>
auto lemon_networksimplex_k_disjoint_shortest_path(CompactDiGraph<Vertex, Weight> &graph, Vertex source,
                                                   Vertex target) -> std::pair<int, std::vector<std::pair<Vertex, Vertex>>> {

    typedef std::pair<Vertex, Vertex> Edge;
    std::vector<Edge> arcs;
    auto edge_it = graph.edges_begin();
    for (auto i = 0; edge_it != graph.edges_end(); edge_it++, i++) {
        auto from = edge_it->from;
        auto to = edge_it->to == target ? edge_it->to : (graph.num_nodes() - 2) + edge_it->to;
        auto arc = std::make_pair(from, to);
        arcs.push_back(arc);
    }
    auto num_arcs = arcs.size();

    for (auto j = 0; j < graph.num_nodes() - 2; j++) {
        auto i = num_arcs + j;
        auto from = (graph.num_nodes() - 2) + (j + 2);
        auto to = (j + 2);
        auto arc = std::make_pair(from, to);
        arcs.push_back(arc);
    }

    lemon::StaticDigraph sg;
    sg.build(graph.num_nodes() * 2 - 2, arcs.begin(), arcs.end());
    lemon::StaticDigraph::ArcMap<Weight> costMap(sg);
    lemon::StaticDigraph::ArcMap<int> upperMap(sg);

    // std::cout << "using lemon" << std::endl;

    edge_it = graph.edges_begin();
    for (auto i = 0; edge_it != graph.edges_end(); edge_it++, i++) {
        upperMap[sg.arc(i)] = 1;
        costMap[sg.arc(i)] = edge_it->w;
    }

    for (auto j = 0; j < graph.num_nodes() - 2; j++) {
        auto i = num_arcs + j;
        upperMap[sg.arc(i)] = 1;
        costMap[sg.arc(i)] = 0;
    }
    lemon::NetworkSimplex<lemon::StaticDigraph> ns(sg);


    auto [e_it, e_end] = graph.out_edges(source);
    ns.upperMap(upperMap).costMap(costMap).stSupply(sg.node(source), sg.node(target), std::distance(e_it, e_end)).run();
//    ns.upperMap(upperMap).costMap(costMap).stSupply(sg.node(source), sg.node(target), 5);
    auto t_start = std::chrono::steady_clock::now();
    ns.run();
    auto t_end = std::chrono::steady_clock::now();
    auto t_dur = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
    spdlog::info("just network_simplex run() took {}ms", t_dur);

    lemon::StaticDigraph::ArcMap<int> flowMap(sg);
    ns.flowMap(flowMap);

    std::vector<Edge> sp_edges;
    edge_it = graph.edges_begin();
    auto k = 0;
    for (auto i = 0; edge_it != graph.edges_end(); edge_it++, i++) {
        auto from = edge_it->from;
        auto to = edge_it->to == target ? edge_it->to : (graph.num_nodes() - 2) + edge_it->to;
        auto flow = flowMap[sg.arc(i)];
        if (flow > 0) {
            sp_edges.emplace_back(edge_it->from, edge_it->to);
            if (from == 0) {
                k += 1;
            }
        }
    }

    return std::make_pair(k, sp_edges);
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
    {
        auto t_start = std::chrono::steady_clock::now();
        auto [k, sp_arcs] = kdisjoint_shortest_path<unsigned int, int>(graph, 0, 1);
        auto t_end = std::chrono::steady_clock::now();
        auto t_dur = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
        spdlog::info("finished suurballe k-disjoint shortest path after {}ms", t_dur);

        // graph weights may have been modified -> reload instance
        graph = load<unsigned int, int>(instance_path);
        auto sum = 0;
        for (auto [u, v]: sp_arcs) {
            sum += graph.get_edge(u, v)->w;
        }
        std::cout << k << "," << sum << "," << t_dur << std::endl;
    }
    {
        auto t_start = std::chrono::steady_clock::now();
        auto [k, sp_arcs] = lemon_costscaling_k_disjoint_shortest_path<unsigned int, int>(graph, 0, 1);
        auto t_end = std::chrono::steady_clock::now();
        auto t_dur = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
        spdlog::info("finished lemon::CostScaling k-disjoint shortest path after {}ms", t_dur);

        // graph weights may have been modified -> reload instance
        graph = load<unsigned int, int>(instance_path);
        auto sum = 0;
        for (auto [u, v]: sp_arcs) {
            sum += graph.get_edge(u, v)->w;
        }
        std::cout << k << "," << sum << "," << t_dur << std::endl;
    }
    {
        auto t_start = std::chrono::steady_clock::now();
        auto [k, sp_arcs] = lemon_networksimplex_k_disjoint_shortest_path<unsigned int, int>(graph, 0, 1);
        auto t_end = std::chrono::steady_clock::now();
        auto t_dur = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
        spdlog::info("finished lemon::NetworkSimplex k-disjoint shortest path after {}ms", t_dur);

        // graph weights may have been modified -> reload instance
        graph = load<unsigned int, int>(instance_path);
        auto sum = 0;
        for (auto [u, v]: sp_arcs) {
            sum += graph.get_edge(u, v)->w;
        }
        std::cout << k << "," << sum << "," << t_dur << std::endl;
    }
    return 0;
}

