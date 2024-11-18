#!/bin/bash

# Check if build directory exists, if not create it
if [ ! -d "build" ]; then
    mkdir build
    cd build
    cmake ..
    make
else
    cd build 
    make 
    cd ../
fi

# Path to the parameters file
PARAMS_FILE="./benchmark/parameters.txt"

# Function to update a parameter in the file
update_param() {
    param_name=$1
    new_value=$2
    sed -i "s/^$param_name=.*/$param_name=$new_value/" "$PARAMS_FILE"
}

change_parameter()
{       param_name=$1
        read -p "Enter new value for $param_name: " new_value
        update_param "$param_name" "$new_value"
}

# Function to handle parameter configuration for pre_target or rand_targets
configure_parameters() {
    mode=$1
    if [ "$mode" == "pre_target" ]; then
        # Read and confirm/update size_of_target and predefined_targets
        size_of_target=$(grep -oP '^size_of_target=\K\d+' "$PARAMS_FILE")
        predefined_targets=$(grep -oP '^predfined_targets=\K.*' "$PARAMS_FILE")
        number_runs=$(grep -oP '^number_runs=\K.*' "$PARAMS_FILE")
        max_num_drones_to_test=$(grep -oP '^max_num_drones_to_test=\K.*' "$PARAMS_FILE")
        echo "For Predefined Targets Benchmark"
        echo "Current number of target: $size_of_target"
        echo "Current predefined targets: $predefined_targets"
        echo "Current number_runs (repeat count due to random topology coverage): $number_runs"
        echo "Current max number of drones to test: $max_num_drones_to_test"
        read -p "Keep current values for benchmark? (y/n):" response
        if [[ $response == "no" || $response == "n" ]]; then
            read -p "Enter the new number of targets: " new_size_of_target
            update_param "size_of_target" "$new_size_of_target"
            size_of_target=$new_size_of_target

            # Collect each pair of target coordinates from the user
            new_targets=""
            for ((i=1; i<=size_of_target; i++)); do
                read -p "Enter target $i (format: x,y): " target
                new_targets+="{${target}}, "
            done

            # Remove trailing comma and space, then update predefined_targets
            new_targets="${new_targets%, }"
            update_param "predfined_targets" "{$new_targets}"

            echo "Updated size of target to $size_of_target."
            echo "Updated predefined targets to {$new_targets}."

            # Confirm or update number_runs and max_num_drones_to_test
            change_parameter "number_runs"
            change_parameter "max_num_drones_to_test"
        fi

    elif [ "$mode" == "rand_targets" ]; then
        echo "For random Targets Benchmark"
        max_number_targets=$(grep -oP '^max_number_targets=\K\d+' "$PARAMS_FILE")
        number_configurations_per_targets=$(grep -oP '^number_configurations_per_targets=\K\d+' "$PARAMS_FILE")
        max_drones_to_check=$(grep -oP '^max_drones_to_check=\K\d+' "$PARAMS_FILE")
        echo "Current max number targets: $max_number_targets"
        echo "Current number configurations per targets set: $number_configurations_per_targets"
        echo "Current max drones to check: $max_drones_to_check"
        read -p "Keep current values for benchmark? (y/n):" response
        if [[ $response == "no" || $response == "n" ]]; then
            # Confirm or update max_number_targets, number_configurations_per_targets, and max_drones_to_check
            change_parameter "max_number_targets"
            change_parameter "number_configurations_per_targets"
            change_parameter "max_drones_to_check"
        fi
    fi
}

# Run benchmark functions
run_benchmark_pre_target() {
    ./benchmark_predefined_targets
    cd ../benchmark
    python plot_benchmark_predefined_targets.py
    echo 'Data for predefined targets are available in benchmark directory and figures are saved in Fig_results directory'
}

run_benchmark_rand_targets() {
    ./benchmark_random_targets
    cd ../benchmark
    python plot_benchmark_random_targets.py
    echo 'Data for random targets are available in benchmark directory and figures are saved in Fig_results directory'
}

if [ "$1" == "pre_target" ]; then
    configure_parameters "pre_target"
    run_benchmark_pre_target

elif [ "$1" == "rand_targets" ]; then
    configure_parameters "rand_targets"
    run_benchmark_rand_targets

elif [ "$1" == "all" ]; then
    # Check if sage exists, if not prompt user to install
    if ! command -v sage &> /dev/null; then
        echo "sigmath is not installed. Please install it and retry."
        exit 1
    fi
    configure_parameters "pre_target"
    configure_parameters "rand_targets"
    # Run all benchmarks and plots
    cd ./benchmark/SAS
    sage experiments.py
    cd ../../build 

    run_benchmark_pre_target
    run_benchmark_rand_targets

    echo 'All figures are saved in Fig_results directory'

else
    echo "Invalid argument. Use pre_target, rand_targets, or all."
    exit 1
fi
