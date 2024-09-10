#include "catch2/catch.hpp"

#include <vector>
#include <kdisjoint.hpp>

TEST_CASE("k-disjoint paths generation") {
    using Vertex = std::size_t;
    using Weight = int;
    using Edge = std::tuple<Vertex, Vertex, Weight>;

    auto create_edge_in_path_fn = [](auto num_vertices) {
        return [=](auto paths, auto from, auto to) {
            const auto offset = num_vertices - 1;
            for(auto it = paths.begin(); it != paths.end(); ++it) {
                if (std::get<0>(*it) == from && std::get<1>(*it) == to) {
                    return true;
                }
            }
            return false;
        };
    };

    SECTION("example from suurballe") {
        const Vertex num_vertices = 8;

        const Vertex A = 0;
        const Vertex B = 1;
        const Vertex C = 2;
        const Vertex D = 3;
        const Vertex E = 4;
        const Vertex F = 5;
        const Vertex G = 6;
        const Vertex Z = 7;

        std::vector<Edge> edges
                {Edge(A, B, 1), Edge(B, C, 1),
                 Edge(C, D, 1), Edge(D, E, 1),
                 Edge(E, Z, 1), Edge(A, F, 1),
                 Edge(F, G, 6), Edge(G, Z, 1),
                 Edge(A, E, 8), Edge(B, Z, 8),
                 Edge(F, C, 2), Edge(D, G, 2)};

        CompactDiGraph<Vertex, Weight> graph = CompactDiGraph(num_vertices, edges);
        auto [k, paths] = kdisjoint_shortest_path(graph, A, Z);
        REQUIRE(k == 3);

        auto edge_in_paths = create_edge_in_path_fn(num_vertices);

        REQUIRE(edge_in_paths(paths, A, B));
        REQUIRE(edge_in_paths(paths, B, Z));

        REQUIRE(edge_in_paths(paths, A, E));
        REQUIRE(edge_in_paths(paths, E, Z));

        REQUIRE(edge_in_paths(paths, A, F));
        REQUIRE(edge_in_paths(paths, F, C));
        REQUIRE(edge_in_paths(paths, C, D));
        REQUIRE(edge_in_paths(paths, D, G));
        REQUIRE(edge_in_paths(paths, G, Z));
    }

    SECTION("example2") {
        const Vertex num_vertices = 7;
        std::vector<Edge> edges
                {Edge(0, 1, 0), Edge(1, 2, 0),
                 Edge(2, 3, -1), Edge(3, 6, 0),
                 Edge(0, 4, -1), Edge(4, 5, 5),
                 Edge(5, 6, -1), Edge(0, 3, 0),
                 Edge(4, 2, -1), Edge(3, 5, -1)};
        std::vector<Weight> weights{0, 0, -1, 0, -1, 5, -1, 0, -1, -1};

        CompactDiGraph<Vertex, Weight> graph = CompactDiGraph(num_vertices, edges);
        auto [k, paths] = kdisjoint_shortest_path(graph, static_cast<Vertex>(0), num_vertices-1);
        REQUIRE(k == 2);

        auto edge_in_paths = create_edge_in_path_fn(num_vertices);

        REQUIRE(edge_in_paths(paths, 0, 1));
        REQUIRE(edge_in_paths(paths, 1, 2));
        REQUIRE(edge_in_paths(paths, 2, 3));
        REQUIRE(edge_in_paths(paths, 3, 6));

        REQUIRE(edge_in_paths(paths, 0, 4));
        REQUIRE(edge_in_paths(paths, 4, 5));
        REQUIRE(edge_in_paths(paths, 5, 6));
    }

    SECTION("example3") {
        const Vertex num_vertices = 11;
        std::vector<Edge> edges
                {Edge(0, 1, 0), Edge(0, 2, 0),
                 Edge(0, 3, 0), Edge(1, 4, 5),
                 Edge(1, 5, -10), Edge(2, 5, 10),
                 Edge(3, 5, -5), Edge(3, 6, 10),
                 Edge(4, 7, 5), Edge(5, 8, 5),
                 Edge(6, 9, 10), Edge(7, 10, 0),
                 Edge(8, 10, 0), Edge(9, 10, 0)};
        std::vector<Weight> weights{0, 0, 0, 5, -10, 10, -5, 10, 5, 5, 10, 0, 0, 0};

        CompactDiGraph<Vertex, Weight> graph = CompactDiGraph(num_vertices, edges);
        auto [k, paths] = kdisjoint_shortest_path(graph, static_cast<Vertex>(0), num_vertices-1);
//        auto [k, paths] = kdisjoint_shortest_path<Vertex, Weight>(num_vertices, edges, weights, 3);
        REQUIRE(k == 3);

        auto edge_in_paths = create_edge_in_path_fn(num_vertices);

        REQUIRE(edge_in_paths(paths, 0, 1));
        REQUIRE(edge_in_paths(paths, 1, 4));
        REQUIRE(edge_in_paths(paths, 4, 7));
        REQUIRE(edge_in_paths(paths, 7, 10));

        REQUIRE(edge_in_paths(paths, 0, 2));
        REQUIRE(edge_in_paths(paths, 2, 5));
        REQUIRE(edge_in_paths(paths, 5, 8));
        REQUIRE(edge_in_paths(paths, 8, 10));

        REQUIRE(edge_in_paths(paths, 0, 3));
        REQUIRE(edge_in_paths(paths, 3, 6));
        REQUIRE(edge_in_paths(paths, 6, 9));
        REQUIRE(edge_in_paths(paths, 9, 10));
    }

    SECTION("example from the paper") {
        const Vertex num_vertices = 6;
        std::vector<Edge> edges
                {Edge(0, 1, 0), Edge(0, 2, 0), Edge(1, 3, -10), Edge(2, 4, -10),
                 Edge(3, 4, -8), Edge(3, 5, 0), Edge(4, 5, 0)};

        CompactDiGraph<Vertex, Weight> graph = CompactDiGraph(num_vertices, edges);
        auto [k, paths] = kdisjoint_shortest_path(graph, static_cast<Vertex>(0), num_vertices-1);
        REQUIRE(k == 2);

        auto edge_in_paths = create_edge_in_path_fn(num_vertices);

        REQUIRE(edge_in_paths(paths, 0, 1));
        REQUIRE(edge_in_paths(paths, 1, 3));
        REQUIRE(edge_in_paths(paths, 3, 5));

        REQUIRE(edge_in_paths(paths, 0, 2));
        REQUIRE(edge_in_paths(paths, 2, 4));
        REQUIRE(edge_in_paths(paths, 4, 5));
    }

    SECTION("example from the paper - try to find more ks then possible") {
        const Vertex num_vertices = 6;
        std::vector<Edge> edges
                {Edge(0, 1, 0), Edge(0, 2, 0), Edge(1, 3, -10), Edge(2, 4, -10),
                 Edge(3, 4, -8), Edge(3, 5, 0), Edge(4, 5, 0)};
        std::vector<Weight> weights{0, 0, -10, -10, -8, 0, 0};

        CompactDiGraph<Vertex, Weight> graph = CompactDiGraph(num_vertices, edges);
        auto [k, paths] = kdisjoint_shortest_path(graph, static_cast<Vertex>(0), num_vertices-1);

        REQUIRE(k == 2);

        auto edge_in_paths = create_edge_in_path_fn(num_vertices);

        REQUIRE(edge_in_paths(paths, 0, 1));
        REQUIRE(edge_in_paths(paths, 1, 3));
        REQUIRE(edge_in_paths(paths, 3, 5));

        REQUIRE(edge_in_paths(paths, 0, 2));
        REQUIRE(edge_in_paths(paths, 2, 4));
        REQUIRE(edge_in_paths(paths, 4, 5));
    }

    SECTION("example with three paths") {
        const Vertex num_vertices = 11;
        std::vector<Edge> edges
                {Edge(0, 1, 0), Edge(0, 2, 0), Edge(0, 3, 0),
                 Edge(1, 4, -10), Edge(2, 4, -5), Edge(2, 5, -4), Edge(3, 5, -10), Edge(3, 6, -2),
                 Edge(4, 7, -2), Edge(4, 8, -1), Edge(5, 8, -2), Edge(5, 9, -10), Edge(6, 9, -2),
                 Edge(7, 10, 0), Edge(8, 10, 0), Edge(9, 10, 0)};

        {
            CompactDiGraph<Vertex, Weight> graph = CompactDiGraph(num_vertices, edges);
            auto [k, paths] = kdisjoint_shortest_path(graph, static_cast<Vertex>(0), num_vertices-1);

            REQUIRE(k == 3);

            auto edge_in_paths = create_edge_in_path_fn(num_vertices);

            REQUIRE(edge_in_paths(paths, 0, 1));
            REQUIRE(edge_in_paths(paths, 1, 4));
            REQUIRE(edge_in_paths(paths, 4, 7));
            REQUIRE(edge_in_paths(paths, 7, 10));

            REQUIRE(edge_in_paths(paths, 0, 2));
            REQUIRE(edge_in_paths(paths, 2, 5));
            REQUIRE(edge_in_paths(paths, 5, 8));
            REQUIRE(edge_in_paths(paths, 8, 10));

            REQUIRE(edge_in_paths(paths, 0, 3));
            REQUIRE(edge_in_paths(paths, 3, 6));
            REQUIRE(edge_in_paths(paths, 6, 9));
            REQUIRE(edge_in_paths(paths, 9, 10));
        }
    }

    SECTION("example with three paths - limiting k") {
        const Vertex num_vertices = 11;
        std::vector<Edge> edges
                {Edge(0, 1, 0), Edge(0, 2, 0), Edge(0, 3, 0),
                 Edge(1, 4, -10), Edge(2, 4, -5), Edge(2, 5, -4), Edge(3, 5, -10), Edge(3, 6, -2),
                 Edge(4, 7, -2), Edge(4, 8, -1), Edge(5, 8, -2), Edge(5, 9, -10), Edge(6, 9, -2),
                 Edge(7, 10, 0), Edge(8, 10, 0), Edge(9, 10, 0)};
        {
            Vertex M = 2;
            CompactDiGraph<Vertex, Weight> graph = CompactDiGraph(num_vertices, edges);
            auto [k, paths] = kdisjoint_shortest_path(graph, static_cast<Vertex>(0), num_vertices-1, M);

            REQUIRE(k == 2);

            auto edge_in_paths = create_edge_in_path_fn(num_vertices);

            REQUIRE(edge_in_paths(paths, 0, 1));
            REQUIRE(edge_in_paths(paths, 1, 4));
            REQUIRE(edge_in_paths(paths, 4, 7));
            REQUIRE(edge_in_paths(paths, 7, 10));

            REQUIRE(edge_in_paths(paths, 0, 3));
            REQUIRE(edge_in_paths(paths, 3, 5));
            REQUIRE(edge_in_paths(paths, 5, 9));
            REQUIRE(edge_in_paths(paths, 9, 10));
        }
    }
}

