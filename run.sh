#!/bin/bash

# Get the value of n from the input argument
for arg in "$@"
do
    case $arg in
        n=*)
        n="${arg#n=}"
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
python plot_data.py $n
