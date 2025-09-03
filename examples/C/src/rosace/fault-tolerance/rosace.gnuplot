# Gnuplot commands for the Rosace.lf program.

set title 'ROSACE'            # Set graph title

set xlabel "Time (seconds)"
 
set terminal pdf size 5, 3.5    # Set the output format to PDF
set output 'rosace.pdf'         # Set output file.

set label "altitude" at 0.2, -1.2

plot 'altitude.data' using 1:2 with lines linetype 1 linewidth 2 title 'altitude'

plot 'airspeed.data' using 1:2 with lines linetype 1 linewidth 2 title 'airspeed'
