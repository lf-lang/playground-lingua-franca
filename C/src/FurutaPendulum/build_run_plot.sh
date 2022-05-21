#!/bin/bash
# This script compiles the generated code,
# executes it, and plots the results.
# The first argument is the name of the main reactor,
# the second is the root name of the gnuplot file to use,
# and the last argument is the viewer to render the PDF in.

# Build the generated code.
cd ${LF_SOURCE_GEN_DIRECTORY}
cmake .
make

# Move the executable to the bin directory.
mv $1 ${LF_BIN_DIRECTORY}

# Invoke the executable.
${LF_BIN_DIRECTORY}/$1

# Plot the results, which have appeared in the src directory.
gnuplot ${LF_SOURCE_DIRECTORY}/$2.gnuplot

# Open the produced PDF using the specified viewer
if ! command -v $3 &> /dev/null
then
    echo "'$3' could not be found; please specify another PDF viewer."
    exit
else
    $3 $2.pdf &
fi
