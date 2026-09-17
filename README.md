# k-disjoint shortest path algorithm in C++

A reimplementation of the k-disjoint shortest path algorithm used in [Schiffer et al. (2021)](https://doi.org/10.1016/j.trb.2020.11.001)
> Schiffer, M., Hiermann, G., Rüdel, F., and Walther, G (2021). A polynomial-time algorithm for user-based relocation in free-floating car sharing systems,
Transportation Research Part B: Methodological 143, 65-85.

to solve a car sharing relocation problem with flexible requests (CSRP-FR) arising in free-floating car sharing systems.
Note that this implementation solves the node-disjoint variant of the k-dSP problem using the procedure proposed by [Suurballe (1978)](https://doi.org/10.1002/net.3230040204).

This branch runs alternative approaches which solve the min-cost-flow problem using the LEMON library, i.e. the CostScaling and NetworkSimplex algorithms.
Initial testing suggest that these work better for larger instances.

### Project structure

- `/benchmark/` small benchmarks using google/benchmark
- `/catch2_tests/` set of unit tests using catch2 to verify basic functionality
- `/lib/lemon` copy of the lemon library v1.3.1 (https://lemon.cs.elte.hu/trac/lemon)
- `/instances/` contains a subset of instances based on the CSRP-FR instances from the artificial benchmark set
- `/include/kdisjoint.hpp` main algorithm and graph implementation 
- `/src/main.cpp` simple cli to run the instance files in `/instances/`


### Installation

This project uses [CMake](https://cmake.org/cmake/help/latest/index.html) buildsystem generator, and [CPM](https://github.com/cpm-cmake/CPM.cmake) for dependency management.
It provides three targets: `kdsp-cpp`, `catch2_tests`, and `benchmarks`. 
The main target is `kdsp-cpp`, which builds a CLI application to solve a kdsp problem for instances located in `/instances/`.

The code uses features from the C++17 standard and was tested on **Ubuntu 20.04 LTS via WSL**, using GNU GCC 9.4.


### How to run

- Linux: `kdsp-cpp -i ./instances/ffcs_bench_wd_B300_SNone_2000jobs_750cars.dat`

This should result in an output similar to the following:
```
[2026-09-17 20:17:43.630] [info] reading instance from ../../kdsp/resources/ffcs_graphs/ffcs_bench_wd_B300_SNone_2000jobs_750cars.dat
[2026-09-17 20:17:43.650] [info] start k-disjoint shortest path
[2026-09-17 20:17:43.795] [info] finished suurballe k-disjoint shortest path after 144ms
706,-17279000,144
[2026-09-17 20:17:43.845] [info] finished lemon::CostScaling k-disjoint shortest path after 29ms
706,-17279000,29
[2026-09-17 20:17:43.886] [info] just network_simplex run() took 9ms
[2026-09-17 20:17:43.886] [info] finished lemon::NetworkSimplex k-disjoint shortest path after 19ms
706,-17279000,19
```

The last lines contains the number of `k`s, the total cost values of the k-shortest paths found, and the runtime in milliseconds.
Note that the implementation can handle negative arc costs; however, the graph must not contain negative cost cycles.


