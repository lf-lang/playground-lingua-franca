[![CI](https://github.com/lf-lang/examples-lingua-franca/actions/workflows/ci.yml/badge.svg)](https://github.com/lf-lang/examples-lingua-franca/actions/workflows/ci.yml)

## 🛝 Lingua Franca Playground
Get to know the [Lingua Franca coordination language](https://lf-lang.org) and browse [example programs](examples/README.md).
To view, edit, and run the programs, you can either run locally on your computer or run in the cloud using either GitHub Codespaces or Gitpod.

## 💻 Running Locally
Quick start:

1. Clone this repo (`git clone git@github.com:lf-lang/playground-lingua-franca.git`)
2. Open with [VS Code](https://code.visualstudio.com) (`code playground-lingua-franca`)
3. Install the [Lingua Franca extension](https://github.com/lf-lang/vscode-lingua-franca) (<kbd>Ctrl</kbd>+<kbd>P</kbd> and enter `ext install lf-lang.vscode-lingua-franca`).

Once in VSCode, navigate to [./examples](./examples) and click on any of the `.lf` files to open them into your editor. To build and run, use <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd> and select `Lingua Franca: Build and Run`. Note: You might need not install additional dependencies in order to successfully build some of the code you find in this repository. For more information, see the [setup-env.bash](./utils/scripts/setup-env.bash) script that we use to configure our Docker-based environments.


## :rocket: Running in the Cloud
Spin up a fully configured dev environment in the cloud that start in seconds.
Any dependencies required for building or running any of the examples are preinstalled.
A web-based VS Code editor, preloaded with the Lingua Franca extension, is accessible through either **GitHub Codespaces** or **GitPod**. Simply click on either of the links below to get started.

[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://github.com/codespaces/new?hide_repo_select=true&repo=477928779&ref=main&skip_quickstart=true&devcontainer_path=.devcontainer%2Fnightly%2Fdevcontainer.json)

[![Open in Gitpod](https://gitpod.io/button/open-in-gitpod.svg)](https://gitpod.io/new#https://github.com/lf-lang/playground-lingua-franca/tree/main)

Once in Codespaces or GitPod, navigate to [./examples](./examples) and click on any of the `.lf` files to open them in the editor. To build and run, use <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd> and select `Lingua Franca: Build and Run`.

#### :wrench: Switch Lingua Franca Build Channels
By default, the Lingua Franca toolchain installed in the container will be the latest nightly version, freshly fetched when the container boots up. This fetching process is handled by a utility script called `setup-lf.bash`, located in the `utils/scripts` directory.

If you wish to switch to a different build channel for Lingua Franca, you can use the following commands in your console:

```bash
# To install the nightly version, built within the last 24 hours
bash ./utils/scripts/setup-lf.bash nightly

# To install the latest stable version
bash ./utils/scripts/setup-lf.bash stable

# To clone the repository and build Lingua Franca from source
bash ./utils/scripts/setup-lf.bash dev
```

Please note that running these commands will remove the current lingua-franca directory.

