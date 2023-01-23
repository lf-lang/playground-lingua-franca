#!/bin/bash
# This script compiles the generated code,
# executes it, and plots the results.
# The first argument is the name of the main reactor
# and the second argument is the viewer to render the PDF in.

# Build the generated code.
cd ${LF_SOURCE_GEN_DIRECTORY}
cmake .
make

# Move the executable to the bin directory.
mv $1 ${LF_BIN_DIRECTORY}

# Move back to source directory to run program
cd ${LF_SOURCE_DIRECTORY}

# Invoke the executable.
${LF_BIN_DIRECTORY}/$1

# Plot the results, which have appeared in the src directory.
gnuplot pendulum.gnuplot

# Open the produced PDF using the specified viewer
if ! command -v $2 &> /dev/null
then
    echo "'$2' could not be found; please specify another PDF viewer."
    exit
else
    $2 pendulum.pdf &
fi
