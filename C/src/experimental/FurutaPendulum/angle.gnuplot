# Gnuplot commands for the FurutaPendulumAngle.lf program.

set title 'Pendulum Angle'      # Set graph title

set xlabel "Time (seconds)"
 
set terminal pdf size 5, 3.5    # Set the output format to PDF
set output 'angle.pdf'         # Set output file.

plot 'theta.data' using 1:2 with lines linetype 1 linewidth 2 title 'theta'