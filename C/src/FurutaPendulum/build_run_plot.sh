#!/bin/bash
# This script compiles the generated code,
# executes it, and plots the results.
# The first argument is the name of the main reactor,
# and the second is the root name of the gnuplot file to use.

# Build the generated code.
cd ${LF_SOURCE_GEN_DIRECTORY}
cmake .
make

# Move the executable to the bin directory.
mv $1 ${LF_BIN_DIRECTORY}

# Invoke the executable.
${LF_BIN_DIRECTORY}/$1

# Plot the results, which have appeared in the src-gen directory.
gnuplot ${LF_SOURCE_DIRECTORY}/$2.gnuplot
open $2.pdf
