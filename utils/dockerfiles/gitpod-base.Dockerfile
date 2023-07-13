FROM gitpod/workspace-full

COPY utils/scripts scripts
RUN bash ./scripts/setup-env.bash
    # Uncomment this line if you want SDKMAN to be installed with Docker.
    # A better way is to run setup-user-env
    # bash -c ". /home/gitpod/.sdkman/bin/sdkman-init.sh && sdk install java 19.0.2-open < /dev/null"
