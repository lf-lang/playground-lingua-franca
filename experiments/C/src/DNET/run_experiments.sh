#!/bin/bash

# Cache sudo credentials at the start
sudo -v

# Keep sudo alive throughout the script
while true; do
    sudo -n true
    sleep 60
    kill -0 "$$" || exit
done 2>/dev/null &

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
base_dir="${script_dir}/../../bin"

# Array of millisecond values
ms_values=(1 2 5 10 50)

# Function to setup network delay
setup_network_delay() {
    local delay=$1
    echo "Setting up network delay of ${delay}ms..."
    
    # Clear any existing traffic control rules
    sudo tc qdisc del dev lo root 2>/dev/null
    
    # Add new delay rule - divide by 2 since loopback gets delayed twice
    sudo tc qdisc add dev lo root netem delay $((delay/2))ms 0ms
    
    # Give the network configuration a moment to settle
    sleep 1
}

# Function to clean up network settings
cleanup_network() {
    echo "Cleaning up network configurations..."
    sudo tc qdisc del dev lo root 2>/dev/null
}

# Process each ms value
for ms in "${ms_values[@]}"; do
    echo "Processing ${ms}ms delay configuration..."
    
    # Setup network delay once for this ms value
    setup_network_delay $ms
    
    # Find and run all executables for this ms value across all test directories
    for test_dir in "$base_dir"/*; do
        if [ -d "$test_dir" ]; then
            ms_dir="$test_dir/${ms}ms"
            if [ -d "$ms_dir" ]; then
                # Find and execute the executable in this ms directory
                # Using find to get the actual executable name
                for executable in "$ms_dir"/*; do
                    if [ -x "$executable" ]; then
                        echo "Running: $executable"
                        pushd "$ms_dir" > /dev/null
                        ./"$(basename "$executable")"
                        popd > /dev/null
                    fi
                done
            fi
        fi
    done
    
    # Clean up network configuration after all executables for this ms value
    cleanup_network
    
    echo "Completed running ${ms}ms delay configuration"
    echo "----------------------------------------"
done