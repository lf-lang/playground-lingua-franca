#!/bin/bash
set -euo pipefail

# This script compiles the generated code,
# executes it, and plots the results.
# The one argument is the name of the main reactor.

# Determine platform specific command to open PDF
openPDF="open"
if [[ "$OSTYPE" =~ ^linux ]]; then
    openPDF="xdg-open"
elif [[ "$OSTYPE" =~ ^cygwin ]]; then
    openPDF="cygstart"
elif [[ "$OSTYPE" =~ ^msys ]]; then
    openPDF="start"
elif [[ "$OSTYPE" =~ ^win ]]; then
    openPDF="start"
fi


# Build the generated code.
cd ${LF_SOURCE_GEN_DIRECTORY}
cmake -DLF_THREADED=1 .
cmake --build .

# Move the executable to the bin directory.
mv $1 ${LF_BIN_DIRECTORY}

# Move back to source directory to run program
cd ${LF_SOURCE_DIRECTORY}

# Invoke the executable.
${LF_BIN_DIRECTORY}/$1

# Plot the results, which have appeared in the src directory.
gnuplot $2

# Open the produced PDF using the specified viewer
if ! command -v $openPDF &> /dev/null
then
    echo "'$openPDF' could not be found; please specify another PDF viewer."
    exit
else
    $openPDF rosace.pdf &
fi
