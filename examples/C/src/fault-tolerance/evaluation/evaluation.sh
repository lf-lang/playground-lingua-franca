# Setup experiment.
NUM=10000
failure_rates=(0.005 0.01 0.02 0.05)

# Make directory named result_dir$NUM (e.g., result_dir100000)
mkdir -p result_dir$NUM

# Determine the timeout value based on NUM
if [ "$NUM" -eq 100000 ]; then
  timeout_value="4200 secs"
elif [ "$NUM" -eq 10000 ]; then
  timeout_value="420 secs"
elif [ "$NUM" -eq 1000 ]; then
  timeout_value="42 secs"
elif [ "$NUM" -eq 100 ]; then
  timeout_value="4200 msecs"
else
  echo "Unsupported NUM value: $NUM"
  exit 1
fi

base_names=(EvalApproach EvalNoAdvance EvalWorstCase)

# Copy, rename, and update file contents
for rate in "${failure_rates[@]}"
do
  rate_int=$(printf "%.0f" $(echo "$rate * 1000" | bc))
  rate_str=$(printf "%04d" $rate_int)

  percent=$(echo "$rate * 100" | bc -l)
  percent_fmt=$(printf "%.1f" $percent)

  for base in "${base_names[@]}"
  do
    new_file="result_dir$NUM/${base}${rate_str}.lf"
    cp "${base}.lf" "$new_file"

    sed -i '' "s/timeout: 420 secs/timeout: ${timeout_value}/g" "$new_file"
    sed -i '' "s/failure_rate *= *[0-9.]*, *\/\/.*/failure_rate = ${rate}, \/\/ ${percent_fmt}% chance/g" "$new_file"

    sed -i '' 's#import Coordinator from "lib/FaultTolerantTaskTemplate.lf.lf"#import Coordinator from "../lib/FaultTolerantTaskTemplate.lf.lf"#' "$new_file"
    sed -i '' 's#import Task1, TaskScheduler from "TaskExampleEval.lf"#import Task1, TaskScheduler from "../TaskExampleEval.lf"#' "$new_file"
    sed -i '' 's#import Task2, TaskScheduler from "TaskExampleEval.lf"#import Task2, TaskScheduler from "../TaskExampleEval.lf"#' "$new_file"
    sed -i '' 's#import Task3, TaskScheduler from "TaskExampleEval.lf"#import Task3, TaskScheduler from "../TaskExampleEval.lf"#' "$new_file"
    sed -i '' 's#import NoTaskDrop from "lib/TaskRetryNoAdvanceTemplate.lf"#import NoTaskDrop from "../lib/TaskRetryNoAdvanceTemplate.lf"#' "$new_file"
  done
done

# Compile all modified files
for rate in "${failure_rates[@]}"
do
  rate_int=$(printf "%.0f" $(echo "$rate * 1000" | bc))
  rate_str=$(printf "%04d" $rate_int)

  for base in "${base_names[@]}"
  do
    file_path="result_dir$NUM/${base}${rate_str}.lf"
    echo "Compiling $file_path ..."
    lfc "$file_path"
  done
done

########### Execution

# Save original directory
START_DIR=$(pwd)

# Move to project root
cd ../../../../

# Detect OS
UNAME=$(uname)

# Execute binaries and collect timing output
for rate in "${failure_rates[@]}"
do
  rate_int=$(printf "%.0f" $(echo "$rate * 1000" | bc))
  rate_str=$(printf "%04d" $rate_int)

  for base in "${base_names[@]}"
  do
    binary="./bin/${base}${rate_str}"
    output="src/fault-tolerance/evaluation/result_dir$NUM/${base}_fail${rate_str}.txt"

    echo "Running $binary ..."

    if [[ "$UNAME" == "Linux" ]]; then
      /usr/bin/time -v "$binary" > "$output" 2>&1
    elif [[ "$UNAME" == "Darwin" ]]; then
      gtime -v "$binary" > "$output" 2>&1
    else
      echo "❌ Unsupported OS: $UNAME"
      exit 1
    fi

    echo "✅ Finished ${base}${rate_str}"
  done
done

# Return to original directory
cd "$START_DIR"
