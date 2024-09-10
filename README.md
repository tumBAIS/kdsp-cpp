# k-disjoint shortest path algorithm in C++

A reimplementation of the k-disjoint shortest path algorithm used in [Schiffer et al. (2021)](https://doi.org/10.1016/j.trb.2020.11.001)
> Schiffer, M., Hiermann, G., Rüdel, F., and Walther, G (2021). A polynomial-time algorithm for user-based relocation in free-floating car sharing systems,
Transportation Research Part B: Methodological 143, 65-85.

to solve a car sharing relocation problem with flexible requests (CSRP-FR) arising in free-floating car sharing systems.
Note that this implementation solves the node-disjoint variant of the k-dSP problem using the procedure proposed by [Suurballe (1978)](https://doi.org/10.1002/net.3230040204).


### Project structure

- `/benchmark/` small benchmarks using google/benchmark
- `/catch2_tests/` set of unit tests using catch2 to verify basic functionality
- `/instances/` contains a subset of instances based on the CSRP-FR instances from the artificial benchmark set
- `/include/kdisjoint.hpp` main algorithm and graph implementation 
- `/src/main.cpp` simple cli to run the instance files in `/instances/`


### Installation

This project uses [CMake](https://cmake.org/cmake/help/latest/index.html) buildsystem generator, and [CPM](https://github.com/cpm-cmake/CPM.cmake) for dependency management.
It provides three targets: `kdsp-cpp`, `catch2_tests`, and `benchmarks`. 
The main target is `kdsp-cpp`, which builds a CLI application to solve a kdsp problem for instances located in `/instances/`.

The code uses features from the C++17 standard and was tested on Windows 11, using MSVC 14.35, and Ubuntu 20.04 LTS via WSL, using GNU GCC 9.4.


### How to run

- Windows: `kdsp-cpp.exe -i ./instances/ffcs_bench_wd_B300_SNone_2000jobs_750cars.dat`
- Linux: `kdsp-cpp -i ./instances/ffcs_bench_wd_B300_SNone_2000jobs_750cars.dat`

This should result an output similar to the following:
```
[2024-09-10 20:14:47.596] [info] reading instance from ./instances/ffcs_bench_wd_B300_SNone_2000jobs_750cars.dat
[2024-09-10 20:14:47.760] [info] start k-disjoint shortest path
[2024-09-10 20:14:48.542] [info] finished k-disjoint shortest path after 781ms
706,-17279000,781
```
The last line contains the number of `k`s, the total cost values of the k-shortest paths found, and the runtime in milliseconds.
Note that the implementation can handle negative arc costs; however, the graph must not contain negative cost cycles.


