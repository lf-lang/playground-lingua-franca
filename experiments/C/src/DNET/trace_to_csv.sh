#!/bin/bash

# BASE_DIR="./bin"

# for dir in "$BASE_DIR"/*; do
# 	if [[ -d "$dir" ]]; then
# 		echo "Trace_to_csv files in $dir:"
# 		for subdir in "$dir"/*; do
# 			if [[ -d "$subdir" ]]; then
# 				echo "Sub Dir is $subdir"
# 				for subsubdir in "$subdir"/*;do
# 					if [[ -d "$subsubdir" ]]; then
# 						echo "Sub Sub Dir is $subsubdir"
# 						for trace in "$subsubdir"/*.lft; do
# 							pushd "$subsubdir" > /dev/null
# 							echo "target file: $(basename "$trace")"
# 							if [[ "$(basename "$dir")" == "Plain" ]]; then
# 								echo "trace_to_csv_plain $(basename "$trace")"
# 								trace_to_csv_plain $(basename "$trace")
# 							else
# 								echo "trace_to_csv_DNET $(basename "$trace")"
# 								trace_to_csv_DNET $(basename "$trace")
# 							fi
# 							popd > /dev/null
# 						done
# 						for summary in "$subsubdir"/*summary.csv; do
# 							file=$(basename "$summary")
# 							new_path=$(echo "$subsubdir" | sed 's|/bin|/bin/summary|')
# 							mkdir -p $new_path
# 							cp $summary "$new_path/$file"
# 						done
# 					fi
# 				done
# 			fi
# 		done
# 	fi
# done

# Array of millisecond values
ms_values=(1 2 5 10 50)

base_dir="/Users/bjun5/projects/lf-lang/lf-pubs/federated/PADS25/Evaluation/bin"

# Now process all rti.lft files
echo "Processing all rti.lft files..."
for test_dir in "$base_dir"/*; do
    if [ -d "$test_dir" ]; then
        for ms_dir in "$test_dir"/*ms; do
            if [ -d "$ms_dir" ] && [ -f "$ms_dir/rti.lft" ]; then
                echo "Processing rti.lft in: $ms_dir"
                pushd "$ms_dir" > /dev/null
                trace_to_csv rti.lft
                popd > /dev/null
            fi
        done
    fi
done