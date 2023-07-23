#!/bin/bash
# This script sets up LF runtime.
set -u

# https://stackoverflow.com/a/7662661
is_valid_sha1() {
    case $1 in
    *[!0-9A-Fa-f]* | "") return 1 ;;
    *)
        case ${#1} in
        40) return 0 ;;
        *) return 2 ;;
        esac
        ;;
    esac
}

RELEASE_BUILD="nightly"
CHECK_INSANE_NAMES=true
REF=""
REPO_URL="https://github.com/lf-lang/lingua-franca.git"

for arg in "$@"; do
    shift
    case "$arg" in
    dev) RELEASE_BUILD="dev" ;;
    stable) RELEASE_BUILD="stable" ;;
    --skip-check-insane-names) CHECK_INSANE_NAMES=false ;;
    --debug) set -x ;;
    --ref=*) REF=${arg#--ref=} ;;
    --custom-repo-url=*) REPO_URL=${arg#--custom-repo-url=} ;;

    esac
done

# Use case here for maximum flexibility if we were to change later
if [ -d "lingua-franca" ]; then
    echo "Directory lingua-franca already exists. Removing it......"
    rm -rf lingua-franca
fi

case "$RELEASE_BUILD" in
'dev')
    BRANCH_TO_FETCH="master"
    SHA=""
    if [ -n "${REF}" ]; then
        is_valid_sha1 "${REF}"
        result=$?
        case $result in
        0) SHA=${REF} ;;
        1)
            BRANCH_TO_FETCH="${REF}"
            ;;
        2)
            if [ $CHECK_INSANE_NAMES = true ]; then
                echo "It appears that you passed in a prefix of SHA-1 as ref."
                echo "Unfortunately, git only allows fetching full 40 character remote SHA-1."
                echo "If you want to fetch a branch with SHA-1-esque name, pass in --skip-check-insane-names as argument."
                echo "I will fetch branch master for you instead."
            else
                BRANCH_TO_FETCH="${REF}"
            fi
            ;;
        esac
    fi
    git clone --branch ${BRANCH_TO_FETCH} --depth 1 "${REPO_URL}" lingua-franca
    pushd lingua-franca || exit 1
    if [ -n "${SHA}" ]; then
        git fetch --depth=1 origin "${SHA}"
        git switch --detach "${SHA}"
    fi
    git submodule update --init --recursive
    ./gradlew assemble
    popd || exit 1
    ;;
*)
    echo "::step:: Getting lf executable......"
    python3 "$(dirname "$0")/get-lf-executable" $RELEASE_BUILD
    mkdir lingua-franca
    # While what we have here is tar.gz, lf release bot appear to have a bug and did not gunzip it.
    # Therefore `tar -xzf` will fail but `tar -xf` will work.
    # Here, we ignore the actual build name (the original name of the file and the original first directory).
    tar -xf lf.tar.gz -C lingua-franca --strip-components 1
    rm lf.tar.gz
    ;;
esac
