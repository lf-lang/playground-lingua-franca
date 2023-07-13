#!/bin/bash

# This script sets up the lingua franca environment; right now, it only works on Debian forks.

set -euxo pipefail

# Check if running Debian fork; the whole script requires apt to properly run.
if [ ! -f "/etc/debian_version" ]; then
    echo "This script requires apt to run, but it appears that this distro is not a debian fork." 1>&2;
    exit 1
fi

# Install dependencies
sudo apt-get update
## Setup C, C++, Python, Rust, protobuf, gRPC
sudo apt-get install --assume-yes \
    build-essential \
    python3 python3-dev python3-pip \
    rustc \
    libprotobuf-dev libprotobuf-c-dev protobuf-c-compiler protobuf-compiler python3-protobuf \
    protobuf-compiler-grpc libgrpc-dev libgrpc++-dev
    
python3 -m pip install --upgrade pip
# Install python dependencies and
# latest CMake; see https://www.kitware.com/cmake-python-wheels/ https://askubuntu.com/a/1070770
python3 -m pip install --exists-action i requests setuptools cmake

INSTALL_USER_DEPENDENCIES=false

for arg in "$@"; do
    shift
    case "$arg" in
        '--install-user-dependencies') INSTALL_USER_DEPENDENCIES=true;;
    esac
done

if [ "$INSTALL_USER_DEPENDENCIES" = true ]; then
    if (command -v "npm" &> /dev/null) ; then
        echo "npm found"
        npm install -g typescript
    else
        echo "npm not found, skipping TS installation. Recommend installing nvm as user."
    fi

    # Check if SDK is installed like what SDKMAN installer does
    set +ux # GitPod will fail if this is not set. TODO: Investigate
    if [ -n "${SDKMAN_DIR:-""}" ] ; then
        echo "SDKMAN found."
        # As SDKMAN is based off shell, enabling u/x will flood output and cause unexpected behaviour
        set +ux
        \. "$SDKMAN_DIR/bin/sdkman-init.sh"
        sdk install java 17.0.7-ms <<< "y"
        sdk use java 17.0.7-ms
        set -ux
    else
        echo "SDKMAN not found, skipping Java installation. Recommend installing SDKMAN as user."
    fi
fi
