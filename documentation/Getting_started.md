title:      Getting Started 
desc:       This guild offers a starting point for building ADORe and running a test scenario.
date:       ${DOC_DATETIME}
version:    ${DOC_VERSION}
template:   document
<<<<<<< HEAD:documentation/Getting_started.md
nav:        Getting Started __3__
=======
nav:        How-To Guides __3__>Getting Started __1__
>>>>>>> 734f10c (updated docs):documentation/getting_started.md
percent:    100
authors:    opensource-ts@dlr.de
           
# Getting Started With ADORe
To use the ADORe build system you must have docker, docker compose, and make 
installed and configured for you user.

### Clone the reposiotry and submodules
```bash
git clone --recurse-submodules -j8 --branch feature/new_build_system git@gitlab.dlr.de:csa/adore.git
```
or if you have already cloned the repository you must initialize the submodules:
```bash
git checkout feature/new_build_system
git submodule update --init --recursive
```

### Building ADORe
Each module provides a Makefile and docker context for build. You can build the 
whole project by navigating to the ADORe project root and running:
```bash
make build
```
You can also build individual modules by navigating to the module and running 
make build such as the following example:
```bash
cd adore_if_ros_msg
make build
```
After building you can run unit tests by running the provided target:
```bash
make test
```

### Static Checking
The ADORe build system provides built-in static checking via cppcheck and cpplint
```bash
make lint
make cppcheck
make lizard
```

Static checks can be run in individual modules by navigating to a module and 
running the provided make targets as in the following example:
```bash
cd sumo_if_ros
make lint
make cppcheck
make lizard
```

### ADORe CLI
The ADORe CLI is a docker compose context with a ros master service, plotlab 
server service 

#### Starting the ADORe CLI
To use the ADORe CLI docker context first build the docker image with the 
provided make target:
```bash
make build_adore-cli
```

Next, run the provided make target to start the ADORe CLI context:
```bash
make adore-cli
```

The ADORe CLI context provides the following features: 
* Execution environment for all ADORe related binaries 
* A pre-generated catkin workspace located at adore/catkin_workspace
* Docker-out-of-docker docker commands can be run within the context
* Reuse of previously generated binaries and build artifacts. All build 
artifacts generated previously with make build can be executed in this 
environment
* Headless, native or windowed plotlab server running as a docker compose 
service.  The display mode for plotlab server can be configured with the 
docker-compose.yaml. For more information on plotlab server please review the
README.md provided by that module at plotlab/README.md
server can be configured 
* ros master running as a docker compose service


Once in the adore-cli context you can launch scenarios with roslaunch:
```bash
cd test/ci_scenarios
roslaunch baseline_test.launch
```
