#!/bin/bash -i
# This script specifically detects and set up nvm and SDKMAN in bash environment, and install
# needed components for LF
set -euxo pipefail

# Check if SDK is installed like what SDKMAN installer does
install_sdk(){
    if (command -v "sdk" &> /dev/null) ; then
        echo "::step:: sdk found in ENV"
    elif [ -n "${SDKMAN_DIR:-}" ] ; then
        echo "::step:: sdk found in SDKMAN_DIR, sourcing......"
        set +ux
        \. "$SDKMAN_DIR/bin/sdkman-init.sh"
        set -ux
    else
        echo "::step:: SDKMAN not found. Installing"
        set +ux
        curl -s "https://get.sdkman.io" | bash
        export SDKMAN_DIR="$HOME/.sdkman"
        \. "$SDKMAN_DIR/bin/sdkman-init.sh"
        set -ux
    fi

    set +ux
    sdk install java 17.0.7-ms <<< "y"
    sdk use java 17.0.7-ms
    set -ux
}

install_nvm(){
    if (command -v "nvm" &> /dev/null) ; then
        echo "::step:: nvm found in ENV"
    elif [ -n "${NVM_DIR:-}" ] ; then
        echo "::step:: nvm found in NVM_DIR, sourcing......"
        set +ux
        [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
        set -ux
    else 
        echo "::step:: nvm not found. Installing"
        set +ux
        curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.3/install.sh | bash
        export NVM_DIR="${HOME}/.nvm"
        [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
        set -ux
        echo "$NVM_DIR/nvm.sh"
    fi

    set +ux
    nvm install --lts
    nvm use --lts
    npm install --global typescript pnpm
}

install_sdk & 
install_nvm &
wait
