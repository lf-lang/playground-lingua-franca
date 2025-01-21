import re

def convert_latex_to_markdown(latex):
    """Convert LaTeX table to Markdown format."""
    # First get content between \begin{tabular} and \end{tabular}
    start = latex.find('\\begin{tabular}')
    end = latex.find('\\end{tabular}')
    if start == -1 or end == -1:
        return "Could not find tabular environment"
    
    content = latex[start:end]
    
    # Split into rows
    rows = []
    current_row = []
    for line in content.split('\n'):
        line = line.strip()
        if not line or line == '\\hline':
            continue
            
        # Handle \multicolumn
        line = line.replace('\\multicolumn{7}{|c||}{\\textbf{ZeroDelayCycle', 'ZeroDelayCycle')
        line = line.replace('\\multicolumn{7}{|c|}{\\textbf{MicrostepDelayCycle', 'MicrostepDelayCycle')
        line = line.replace('\\multicolumn{1}{|r|}{}', '')
        line = line.replace('\\multicolumn{6}{c||}{Timer Period}', 'Timer Period')
        line = line.replace('\\multicolumn{6}{c|}{Timer Period}', 'Timer Period')
        
        # Clean up the line
        line = line.replace('\\\\', '')
        line = line.replace('\\begin{tabular}{|l|rrrrrr||l|rrrrrr|}', '')
        
        # Split cells
        cells = [cell.strip() for cell in line.split('&')]
        if cells and cells[0]:  # Only add non-empty rows
            rows.append('| ' + ' | '.join(cells) + ' |')
    
    # Add separator after the header
    if len(rows) >= 3:
        num_cols = len(rows[2].split('|')) - 1  # Count number of columns
        separator = '|' + '|'.join(['---' for _ in range(num_cols-1)]) + '|'
        rows.insert(3, separator)
    
    return '\n'.join(rows)

# Test data
latex_table = r'''\subfloat[Results from the WiFi setup with an average network round trip time of 8.681]{
	\begin{tabular}{|l|rrrrrr||l|rrrrrr|}
		\hline
		\multicolumn{7}{|c||}{\textbf{Consistency in~\figurename~\ref{fig:Consistency}}} & \multicolumn{7}{|c|}{\textbf{ConsistencyMicrostepDelay in~\figurename~\ref{fig:ConsistencyMicrostepDelay}}} \\
		\hline
		\multicolumn{1}{|r|}{} & \multicolumn{6}{c||}{Timer Period} & \multicolumn{1}{|r|}{} & \multicolumn{6}{c|}{Timer Period} \\
		\hline
		Timer ticks & 15 ms & 30 ms & 45 ms & 75 ms & 150 ms & 500 ms & Timer ticks & 15 ms & 30 ms & 45 ms & 75 ms & 150 ms & 500 ms \\
		\hline
		1-50 & 222.53 & 76.72 & 17.42 & 20.00 & 20.69 & 21.72 & 1-50 & 251.38 & 149.39 & 19.36 & 19.06 & 24.74 & 21.98 \\
		51-100 & 643.15 & 148.42 & 22.63 & 20.55 & 19.98 & 20.55 & 51-100 & 714.00 & 198.56 & 17.27 & 19.36 & 28.37 & 21.02 \\
		101-150 & 1046.61 & 204.69 & 19.48 & 20.64 & 20.06 & 22.82 & 101-150 & 1207.10 & 211.20 & 20.06 & 21.79 & 23.26 & 20.42 \\
		151-200 & 1512.75 & 238.76 & 22.24 & 20.38 & 20.83 & 21.65 & 151-200 & 1713.32 & 253.03 & 21.39 & 22.97 & 22.12 & 20.69 \\
		201-250 & 1926.49 & 262.23 & 24.46 & 23.35 & 20.31 & 21.22 & 201-250 & 2226.49 & 307.58 & 20.72 & 18.91 & 21.66 & 19.73 \\
		251-300 & 2361.49 & 256.17 & 22.56 & 20.50 & 21.34 & 19.93 & 251-300 & 2734.57 & 374.20 & 19.30 & 18.19 & 22.45 & 19.49 \\
		301-350 & 2791.15 & 305.43 & 34.52 & 21.36 & 20.41 & 21.04 & 301-350 & 3234.62 & 347.59 & 27.26 & 21.41 & 22.15 & 20.97 \\
		351-400 & 3286.46 & 334.25 & 57.42 & 20.35 & 19.67 & 19.98 & 351-400 & 3761.64 & 295.61 & 25.09 & 22.05 & 22.15 & 19.69 \\
		401-450 & 3847.32 & 349.14 & 108.66 & 22.12 & 20.69 & 20.05 & 401-450 & 4352.15 & 252.32 & 25.53 & 20.51 & 21.70 & 20.33 \\
		451-500 & 4391.91 & 349.15 & 96.80 & 48.09 & 21.08 & 21.46 & 451-500 & 4919.65 & 191.63 & 27.32 & 23.00 & 24.14 & 22.09 \\
		\hline
	\end{tabular}'''

# Run the conversion
print(convert_latex_to_markdown(latex_table))