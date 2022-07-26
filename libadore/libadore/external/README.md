# libadore external libraries
This project contains all external libraries consumed by libadore not available
in central repositories

This project contains submodules and a Dockerfile necessary to build and package
all external libadore libraries.

## Getting Started
Docker and Make are required

### build
to build all external libraries run make:
```bash
make
```
you can build specific libraries with the provided make targets for example:
```bash
make build_xodr
```

### Build Artifacts
Once a particular library is built artifacts can be found at:
- <library>/build
- <library>/build/<library name>.deb
- <library>/build/install - cmake install context
