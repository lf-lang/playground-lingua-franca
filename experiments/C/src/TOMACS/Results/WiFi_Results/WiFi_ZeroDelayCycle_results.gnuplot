#!/usr/bin/gnuplot

# set terminal pdfcairo enhanced crop color size 40.0,30.0 solid linewidth 4 font "Helvetica, 270"
set key autotitle columnhead
set terminal pdfcairo 
set output 'LagWiFi.pdf'
# ZDC = "#99ffff"; MicrostepDelay = "#4671d5";

set xlabel "Timer Period (ms)"
set ylabel "Lag (ms)"

set style fill solid 0.5 border -1
set style boxplot outliers pointtype 2
set style data boxplot

set datafile separator ","

set xtics ('15' 2.5, '30' 4.5, '45' 6.5, '75' 8.5, '150' 10.5, '500' 12.5)
plot for [i=1:6] 'WiFi_ZeroDelayCycle_results.csv' using (2*i):i lt 1 title (i==1 ? 'Zero Delay Cycle' : ''),\
for [i=1:6] 'WiFi_MicrostepDelayCycle_results.csv' using (2*i + 1):i lt 2 title (i==1 ? 'Microstep Delay Cycle' : '')