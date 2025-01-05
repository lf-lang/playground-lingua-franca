#!/bin/bash

# Cache sudo credentials at the start
sudo -v

# Keep sudo alive throughout the script
while true; do
    sudo -n true
    sleep 60
    kill -0 "$$" || exit
done 2>/dev/null &

# Get the directory where the script is located
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
bin_dir="$(cd "$script_dir" && cd ../../bin && pwd)"

# Array of millisecond values
ms_values=(1 2 5 10 50)

# First phase: Compile .lf files and create directory structure
echo "Phase 1: Compiling .lf files and creating directory structure..."
for folder in */; do
    folder=${folder%/}
    if [[ -d "$folder" ]]; then
        echo "Compile .lf files in $folder:"
        for file in "$folder"/*.lf; do
            if [[ -f "$file" ]]; then
                $lfc "$file"

                if [[ $? -eq 0 ]]; then
                    filename_with_ext="${file##*/}"
                    filename="${filename_with_ext%.lf}"
                    echo "File name is $filename."

                    # Create the folder structure in bin/$folder first
                    mkdir -p "$bin_dir/$folder"

                    # Move the compiled file to the folder
                    mv "$bin_dir/$filename" "$bin_dir/$folder/"

                    # Create copies for each millisecond subfolder
                    for ms in "${ms_values[@]}"; do
                        target_dir="$bin_dir/$folder/${ms}ms"
                        mkdir -p "$target_dir"
                        cp "$bin_dir/$folder/$filename" "$target_dir/$filename"
                        echo "Copied to bin/$folder/${ms}ms/$filename"
                    done

                    # Clean up the original copied file
                    rm "$bin_dir/$folder/$filename"
                    echo "Compiled $file successfully and copied to all ms directories"
                else
                    echo "Failed to compile $file."
                fi
            else
                echo "No .lf files found in $folder."
            fi
        done
    else
        echo "Directory $folder does not exist."
    fi
done

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

# Second phase: Run executables with network delays
echo -e "\nPhase 2: Running executables with network delays..."
for ms in "${ms_values[@]}"; do
    echo "Processing ${ms}ms delay configuration..."
    
    # Setup network delay once for this ms value
    setup_network_delay $ms
    
    # Find and run all executables for this ms value across all test directories
    for test_dir in "$bin_dir"/*; do
        if [ -d "$test_dir" ]; then
            ms_dir="$test_dir/${ms}ms"
            if [ -d "$ms_dir" ]; then
                # Find and execute the executable in this ms directory
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