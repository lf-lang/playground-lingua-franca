#!/bin/bash

# This script sets up the lingua franca environment; right now, it only works on Debian forks.

set -euo pipefail

CROW_URL="https://github.com/CrowCpp/Crow/releases/download/v1.0%2B5/crow-v1.0+5.deb"

# Check if running Debian fork; the whole script requires apt to properly run.
if [ ! -f "/etc/debian_version" ]; then
    echo "This script requires apt to run, but it appears that this distro is not a debian fork." 1>&2;
    exit 1
fi

SETUP_NETWORK=true
SETUP_ROS=true
ROS_USE_OFFICIAL_REPO=true
INSTALL_USER_DEPENDENCIES=false

for arg in "$@"; do
    shift
    case "$arg" in
        '--install-user-dependencies') INSTALL_USER_DEPENDENCIES=true;;
        '--no-setup-network') SETUP_NETWORK=false;;
        '--no-setup-ros') SETUP_ROS=false;;
        '--ros-no-use-official-repo') ROS_USE_OFFICIAL_REPO=false;;
        '--debug') set -x;; 
    esac
done

# Install dependencies

# We need python3.10 to use LF
sudo add-apt-repository -y 'ppa:deadsnakes/ppa'
sudo apt-get update

## Setup C, C++, Python, Rust, protobuf, gRPC, gnuplot
sudo apt-get install --assume-yes \
    build-essential \
    python3.10 python3.10-dev \
    rustc cargo \
    libprotobuf-dev libprotobuf-c-dev protobuf-c-compiler protobuf-compiler python3-protobuf \
    protobuf-compiler-grpc libgrpc-dev libgrpc++-dev gnuplot
    
python3.10 -m pip install --upgrade pip
# Install python dependencies and
# latest CMake; see https://www.kitware.com/cmake-python-wheels/ https://askubuntu.com/a/1070770
sudo python3.10 -m pip install --exists-action i requests setuptools cmake

if [ $SETUP_NETWORK = true ]; then 
    # Install support for protocol buffers
    sudo apt-get install --assume-yes \
        libprotobuf-dev libprotobuf-c-dev protobuf-c-compiler protobuf-compiler \
        python3-protobuf protobuf-compiler-grpc libgrpc-dev libgrpc++-dev
    # Install libwebsockets library
    git clone https://github.com/warmcat/libwebsockets.git
    pushd libwebsockets
    mkdir build
    cd build
    cmake ..
    make
    popd
    # test
    # Install crow
    curl --proto '=https' --tlsv1.2 -L -o ./crow.deb "${CROW_URL}"
    sudo apt-get install --assume-yes ./crow.deb
    rm ./crow.deb
    # Install support for MQTT
    sudo apt-get install --assume-yes mosquitto libmosquitto-dev libpaho-mqtt-dev
fi

if [ $SETUP_ROS = true ]; then
    if [ $ROS_USE_OFFICIAL_REPO = true ]; then
        echo "If you are running this script to setup LF environment yourself, we recommend that you install ROS2 yourself."
        
        OS_VERSION_CODENAME=$(\. /etc/os-release && echo "${VERSION_CODENAME}")
        OS_ID="$(\. /etc/os-release && echo "${ID}")"
        OS_VERSION_ID="$(\. /etc/os-release && echo "${VERSION_ID}")"
        # See https://www.ros.org/reps/rep-2000.html
        ROS_VERSION_CODENAME=""
        if [[ "${OS_ID}" = "ubuntu" ]] && [[ ! "${OS_VERSION_ID}" < "24.04" ]]; then
            ROS_VERSION_CODENAME="jazzy"
        elif ([[ "${OS_ID}" = "ubuntu" ]] && [[ ! "${OS_VERSION_ID}" < "22.04" ]]) || \
        ([[ "${OS_ID}" = "debian" ]] && [[ "${OS_VERSION_ID}" = "11" ]]); then
            ROS_VERSION_CODENAME="iron"
        elif [[ "${OS_ID}" = "ubuntu" ]] && [[ ! "${OS_VERSION_ID}" < "20.04" ]]; then
            ROS_VERSION_CODENAME="foxy"
            echo "Warning: installing an EOL ROS distribution"
        fi

        if [ -n "${ROS_VERSION_CODENAME}" ]; then
            # http://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html
            sudo apt-get install --assume-yes software-properties-common locales

            if ! (locale | grep -e 'utf8' -e 'UTF-8') >/dev/null 2>&1; then 
                sudo locale-gen C.UTF-8
                sudo update-locale LC_ALL=C.UTF-8 LANG=C.UTF-8
                export LANG=C.UTF-8 
            fi

            # Check if Ubuntu by getting ubuntu codename first
            if [ "${OS_ID}" != "ubuntu" ] ; then 
                echo "This script has only been tested on ubuntu. Proceed with caution."
            else
                # On Ubuntu, we need to add universe; 
                sudo add-apt-repository universe --yes
            fi
            
            sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
            echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${OS_VERSION_CODENAME} main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
            sudo apt-get update
            sudo apt-get install --assume-yes "ros-${ROS_VERSION_CODENAME}-desktop" ros-dev-tools
            # Check https://unix.stackexchange.com/a/156326
            echo "$(set -- /opt/ros/*/setup.bash; printf "\. %s" "$1")" >> ~/.bashrc
            realpath ~/.bashrc
        else 
            echo "Your OS version is too old; please consider upgrading or installing ROS yourself."
        fi
    else
        sudo apt-get install --assume-yes ros-desktop-full-dev
    fi
fi

if [ "$INSTALL_USER_DEPENDENCIES" = true ]; then
    if (command -v "npm" &> /dev/null) ; then
        echo "npm found"
        npm install -g typescript
    else
        echo "npm not found, skipping TS installation. Recommend installing nvm as user."
    fi

    # Check if SDK is installed like what SDKMAN installer does
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
