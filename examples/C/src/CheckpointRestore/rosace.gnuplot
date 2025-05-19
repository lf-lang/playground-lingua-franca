# Gnuplot commands for the Rosace.lf program.

set terminal pdf size 5, 3.5    # Set output format to PDF
set output 'rosace.pdf'         # Set output file

set title 'ROSACE'
set xlabel "Time (seconds)"

# First plot: Altitude (in meters)
set ylabel "Altitude (m)"
plot 'altitude.data' using 1:2 with lines linetype 1 linewidth 2 title 'altitude'

# Second plot: Airspeed (in meters per second)
set ylabel "Airspeed (m/s)"
plot 'airspeed.data' using 1:2 with lines linetype 1 linewidth 2 title 'airspeed'