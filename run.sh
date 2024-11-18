#!/bin/bash

# Initialize variables
n=""
a="20"
# Get the value of n from the input argument
for arg in "$@"
do
    case $arg in
        n=*)
        n="${arg#*=}"
        ;;
        a=*)
        a="${arg#*=}"
        ;;
        *)
        echo "Invalid argument: $arg"
        exit 1
        ;;
    esac
done


# Path to the parameters file
PARAMS_FILE="./benchmark/parameters.txt"

# Function to update a parameter in the file
update_param() {
    param_name=$1
    new_value=$2
    sed -i "s/^$param_name=.*/$param_name=$new_value/" "$PARAMS_FILE"
}

# Read size_of_target and predfined_targets from parameters.txt
size_of_target=$(grep -oP '^size_of_target=\K\d+' "$PARAMS_FILE")
predefined_targets=$(grep -oP '^predfined_targets=\K.*' "$PARAMS_FILE")

# Display current values and ask the user for confirmation
echo "Current size_of_target: $size_of_target"
echo "Current predefined_targets: $predefined_targets"
read -p "Are these the values you want? (yes/no): " response

# If the user says no, proceed with further steps
if [[ $response == "no" || $response == "n" ]]; then
    read -p "Do you want random targets? (yes/no): " choice

    if [[ $choice == "yes" || $choice == "y" ]]; then
        # Set random targets to true in parameters file
        update_param "predfined_targets_random" "true"
        echo "Updated predfined_targets_random to true."
    else
        # Set random targets to false in parameters file
        update_param "predfined_targets_random" "false"

        # Ask the user for a new number of targets
        read -p "Enter the number of targets: " new_size_of_target
        update_param "size_of_target" "$new_size_of_target"

        # Collect each pair of target coordinates from the user
        new_targets=""
        for ((i=1; i<=new_size_of_target; i++)); do
            read -p "Enter target $i (format: x,y): " target
            new_targets+="{${target}}, "
        done

        # Remove trailing comma and space, then update predefined_targets
        new_targets="${new_targets%, }"
        update_param "predfined_targets" "{$new_targets}"

        echo "Updated size_of_target to $new_size_of_target."
        echo "Updated predfined_targets to {$new_targets}."
    fi
else
    echo "No changes were made to the parameters file."
fi


# Check if build directory exists, if not create it
if [ ! -d "build" ]
then
    mkdir build
fi

# Update expansion.h with the new value of a
if [ -f "include/expansion.h" ]; then
    # Create a temporary file
    tmp_file=$(mktemp)

    # Replace the line containing '#define a' with the new value
    awk -v a_value="$a" '/#define a/{print "#define a " a_value; next} 1' include/expansion.h > "$tmp_file"

    # Replace the original file with the temporary file
    mv "$tmp_file" include/expansion.h
else
    echo "File include/expansion.h not found."
    exit 1
fi


# Change to build directory and run cmake ..
cd build
cmake ..

# If cmake succeeds, run make and the executable with the value of n
if [ $? -eq 0 ]
then
    make
    ./main n=$n
fi

# Return to the directory containing the script
cd ..

# Run the python script
python plot_data.py "$n" "$a"
