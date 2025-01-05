#!/bin/bash

# Get the directory where the script is located
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
bin_dir="$(cd "$script_dir" && cd ../../bin && pwd)"

# Array of millisecond values
ms_values=(1 2 5 10 50)

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