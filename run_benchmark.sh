#!/bin/bash
# Check if build directory exists, if not create it
if [ ! -d "build" ]
then
    mkdir build
    cd build
    cmake ..
fi

if [ "$1" == "pre_target" ]; then
    # Run benchmark with predefined targets
    cd ./build
    make
    ./benchmark_predefined_targets
    cd ../benchmark
    echo -e "\n\n"
    python plot_benchmark_predefined_targets.py
    echo 'Data are available in benchmark directory and corresponding figures are saved in Fig_results directory'


elif [ "$1" == "rand_targets" ]; then
    # Run benchmark with random targets
    cd ./build
    make
    ./benchmark_random_targets
    cd ../benchmark
    echo -e "\n\n"
    python plot_benchmark_random_targets.py
    echo 'Data are available in benchmark directory and corresponding figures are saved in Fig_results directory'

elif [ "$1" == "all" ]; then
    # Check if sigmath exists, if not prompt user to install
    if ! command -v sigmath &> /dev/null; then
        echo "sigmath is not installed. Please install it and retry."
        exit 1
    fi
    # Run all benchmarks and plots
    cd ./benchmark/SAS
    sage experiments.py
    cd ../../build
    make
    ./benchmark_predefined_targets
    ./benchmark_random_targets
    cd ../benchmark
    python plot_benchmark_predefined_targets.py
    python plot_benchmark_random_targets.py
    echo 'All figures are saved in Fig_results directory'
else
    echo "Invalid argument. Use pre_target, rand_targets, or all."
    exit 1
fi
