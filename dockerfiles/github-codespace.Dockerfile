# While universal:focal is also okay to use, it lacks libpython3.10-dev, but uses python3.10 by default.
FROM mcr.microsoft.com/devcontainers/base:jammy

COPY .scripts .scripts
RUN bash ./.scripts/setup-env.bash
    # GitHub Codespaces container specific Java commands
    # Uncomment this line if you want SDKMAN to be installed with Docker.
    # A better way is to run setup-user-env
    # bash -c "source /usr/local/sdkman/bin/sdkman-init.sh && sdk install java 19.0.2-open < /dev/null"
