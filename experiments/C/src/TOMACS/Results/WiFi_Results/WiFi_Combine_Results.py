import os
import pandas as pd
import re
from typing import List, Dict, Tuple
import glob

def extract_ms_value(folder_name: str) -> int:
    """Extract millisecond value from folder name."""
    match = re.search(r'(\d+)ms', folder_name)
    return int(match.group(1)) if match else 0

def get_csv_paths(base_path: str) -> Dict[str, List[str]]:
    """
    Get all CSV file paths organized by program type.
    Returns a dictionary with program names as keys and lists of CSV paths as values.
    """
    program_paths = {}
    
    # Get all immediate subdirectories (folders)
    folders = [d for d in os.listdir(base_path) 
              if os.path.isdir(os.path.join(base_path, d))]
    
    for folder in folders:
        if folder.startswith(('Cable_', 'WiFi_')):
            program_paths[folder] = []
            folder_path = os.path.join(base_path, folder)
            
            # Get all CSV files recursively
            csv_files = glob.glob(os.path.join(folder_path, '**', '*.csv'), 
                                recursive=True)
            program_paths[folder] = csv_files
            
    return program_paths

def combine_subfolder_csvs(csv_paths: List[str]) -> Dict[str, pd.DataFrame]:
    """
    Combine CSV files with the same name within subfolders by averaging.
    Returns a dictionary with timing (e.g., '1ms') as key and combined DataFrame as value.
    """
    # Group CSV paths by timing and base filename
    grouped_paths: Dict[str, Dict[str, List[str]]] = {}
    
    for path in csv_paths:
        dir_name = os.path.dirname(path)
        timing = re.search(r'_(\d+ms)', dir_name).group(1)
        base_name = os.path.basename(path)
        
        if timing not in grouped_paths:
            grouped_paths[timing] = {}
        if base_name not in grouped_paths[timing]:
            grouped_paths[timing][base_name] = []
            
        grouped_paths[timing][base_name].append(path)
    
    # Combine CSVs for each timing and base filename
    combined_dfs: Dict[str, pd.DataFrame] = {}
    
    for timing, file_groups in grouped_paths.items():
        combined_dfs[timing] = {}
        for base_name, paths in file_groups.items():
            # Read and combine CSVs
            dfs = []
            for path in paths:
                try:
                    df = pd.read_csv(path, header=None)
                    dfs.append(df)
                except Exception as e:
                    print(f"Error reading {path}: {e}")
                    continue
            
            if not dfs:
                print(f"No valid data found for {timing} - {base_name}")
                continue
                
            combined_df = pd.concat(dfs, axis=1).mean(axis=1)
            
            # Store with the original base name (without path)
            base_without_ext = os.path.splitext(base_name)[0]
            combined_dfs[timing][base_without_ext] = combined_df
    
    return combined_dfs

def create_final_results(folder_name: str, combined_dfs: Dict[str, Dict[str, pd.DataFrame]]) -> None:
    """
    Create final results CSV files by appending combined CSVs.
    For Feedback folders, creates two separate result files for Controller and PhysicalPlant.
    The values in the final results are divided by 1,000,000.
    """
    # Determine if this is a Feedback folder
    is_feedback = 'Feedback' in folder_name
    
    # Sort timings
    timings = sorted(combined_dfs.keys(), key=lambda x: extract_ms_value(x))
    
    if is_feedback:
        # Handle Feedback folders (with Controller and PhysicalPlant files)
        controller_df = pd.DataFrame()
        plant_df = pd.DataFrame()
        
        for timing in timings:
            # Find the Controller and PhysicalPlant data
            for base_name, data in combined_dfs[timing].items():
                if 'Controller' in base_name:
                    controller_df[timing] = data / 1000000  # Divide by 1,000,000
                elif 'PhysicalPlant' in base_name:
                    plant_df[timing] = data / 1000000  # Divide by 1,000,000
        
        # Save Controller results
        controller_output = f"{folder_name}_Controller_results.csv"
        controller_df.to_csv(controller_output, index=False)
        
        # Save PhysicalPlant results
        plant_output = f"{folder_name}_PhysicalPlant_results.csv"
        plant_df.to_csv(plant_output, index=False)
        
    else:
        # Handle non-Feedback folders
        result_df = pd.DataFrame()
        
        for timing in timings:
            # Get the first (and should be only) DataFrame for this timing
            data = next(iter(combined_dfs[timing].values()))
            result_df[timing] = data / 1000000  # Divide by 1,000,000
        
        # Save results
        output_filename = f"{folder_name}_results.csv"
        result_df.to_csv(output_filename, index=False)

def process_csvs(base_path: str) -> None:
    """Main function to process all CSV files."""
    # Get all CSV paths organized by program
    program_paths = get_csv_paths(base_path)
    
    # Process each program's CSVs
    for folder_name, csv_paths in program_paths.items():
        if csv_paths:  # Only process if there are CSV files
            # Combine CSVs within subfolders
            combined_dfs = combine_subfolder_csvs(csv_paths)
            
            # Create final results
            if combined_dfs:
                create_final_results(folder_name, combined_dfs)

if __name__ == "__main__":
    # Use current directory as base path
    base_path = "."
    process_csvs(base_path)