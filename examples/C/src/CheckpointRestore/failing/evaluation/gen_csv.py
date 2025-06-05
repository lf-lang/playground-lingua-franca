import os
import csv
import re
from datetime import timedelta

input_dir = 'res100k'
output_csv = 'res100k.csv'

def parse_time_to_seconds(time_str):
    # Handles formats like '7:00.26' (m:ss.xx) and '7:07:31.26' (h:mm:ss.xx)
    try:
        parts = time_str.strip().split(":")
        if len(parts) == 2:
            minutes, seconds = parts
            return int(minutes) * 60 + float(seconds)
        elif len(parts) == 3:
            hours, minutes, seconds = parts
            return int(hours) * 3600 + int(minutes) * 60 + float(seconds)
    except ValueError:
        return 0.0  # fallback if parsing fails

data_rows = []


def custom_sort_key(fname):
    match = re.match(r'(Eval\w+_fail)(\d+)\.txt', fname)
    if match:
        prefix = match.group(1)
        num = int(match.group(2))
        return (prefix, num)
    return (fname, 0)

for filename in sorted(os.listdir(input_dir), key=custom_sort_key):
    if filename.endswith(".txt"):
        with open(os.path.join(input_dir, filename), 'r') as file:
            deadline_miss = 0
            execution_failed = 0
            user_time = 0.0
            system_time = 0.0
            elapsed_time = 0.0

            for line in file:
                if "Deadline missed:" in line:
                    deadline_miss = int(float(line.split(":")[-1].strip()))
                elif "Execution failed: " in line:
                    execution_failed = int(float(line.split(":")[-1].strip()))
                elif "User time (seconds):" in line:
                    user_time = float(line.split(":")[-1].strip())
                elif "System time (seconds):" in line:
                    system_time = float(line.split(":")[-1].strip())
                elif "Elapsed (wall clock) time (h:mm:ss or m:ss):" in line:
                    time_part = line.split("(h:mm:ss or m:ss):")[-1].strip()
                    elapsed_time = parse_time_to_seconds(time_part)

            total_cpu_time = user_time + system_time
            utilization = (total_cpu_time / elapsed_time) * 100 if elapsed_time > 0 else 0.0
            data_rows.append([deadline_miss, execution_failed, f"{utilization:.2f}"])

with open(output_csv, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['Deadline_miss', 'Execution_failed', 'Utilization'])
    writer.writerows(data_rows)

print(f"Data written to {output_csv}")