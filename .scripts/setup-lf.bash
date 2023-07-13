#!/bin/bash
# This script sets up LF runtime.
set -euxo pipefail

RELEASE_BUILD="nightly"
EXAMPLE=0

for arg in "$@"; do
    shift
    case "$arg" in
        'dev') RELEASE_BUILD="dev";;
        'stable') RELEASE_BUILD="stable";;
        '--example') EXAMPLE=1;;
    esac
done


# Use case here for maximum flexibility if we were to change later
if [ ! -d "lingua-franca" ]; then
    case "$RELEASE_BUILD" in
        'dev') 
            git clone https://github.com/lf-lang/lingua-franca.git --branch master --depth 1
            cd lingua-franca
            git submodule update --init --recursive
            ./gradlew buildAll
            cd .. 
        ;;
        *) 
            echo "::step:: Getting lf executable......"
            python3 ./.scripts/get-lf-executable $RELEASE_BUILD
            mkdir lingua-franca
            # While what we have here is tar.gz, lf release bot appear to have a bug and did not gunzip it.
            # Therefore `tar -xzf` will fail but `tar -xf` will work.
            # Here, we ignore the actual build name (the original name of the file and the original first directory). 
            tar -xf lf.tar.gz -C lingua-franca --strip-components 1
            rm lf.tar.gz
        ;;
    esac
else
    echo "::step:: Directory lingua-franca already exist. Ignoring fetch."
fi

if [ $EXAMPLE -eq 1 ] && [ ! -d "examples" ] ; then
    echo "::step:: Cloning examples......"
    git clone https://github.com/lf-lang/examples-lingua-franca.git examples --branch main
fi
