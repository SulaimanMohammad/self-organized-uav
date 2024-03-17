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
