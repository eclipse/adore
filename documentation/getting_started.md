<!--
********************************************************************************
* Copyright (C) 2017-2020 German Aerospace Center (DLR). 
* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
*
* This program and the accompanying materials are made available under the 
* terms of the Eclipse Public License 2.0 which is available at
* http://www.eclipse.org/legal/epl-2.0.
*
* SPDX-License-Identifier: EPL-2.0 
*
* Contributors: 
*   Andrew Koerner 
********************************************************************************
-->
# Getting started with ADORe
To use the ADORe build system you must have docker, docker compose, and make 
installed and configured for you user.
To check this run the following commands:
1. check your docker version:
```bash
$ docker --version
Docker version 20.10.17, build 100c701
```
```bash
$ docker compose version
Docker Compose version v2.6.0
```
2. Check that you are a member of the docker group:
```bash
id | grep docker
...,998(docker),...
```
3. Check your storage and be sure you have ~18GB free:
```bash
df -h
```
### Clone the repository
```bash
git clone git@github.com:eclipse/adore.git
cd adore
git submodule update --init
```
or if you have already cloned the repository you must initialize the submodules:
```bash
git submodule update --init
```

### ADORe command line interface (CLI)
The ADORe CLI is a docker compose context with a ros master service and a 
plotlab server service. 
To use the ADORe CLI docker context run the provided make target to start the 
ADORe CLI context:
```bash
make cli
```

On first run of the ADORe cli the system will be built. Initial build can take 
10-15 minutes depending on system and network. 

The ADORe CLI context provides the following features: 
* Execution environment for all ADORe related binaries 
* A pre-generated catkin workspace located at adore/catkin_workspace
* Reuse of previously generated binaries and build artifacts. All build 
artifacts generated previously with make build can be executed in this 
environment
* Headless, native or windowed plotlab server running as a docker compose 
service.  The display mode for plotlab server can be configured with the 
docker-compose.yaml. For more information on plotlab server please review the
README.md provided by that module at plotlabserver/README.md
server can be configured 
* ros master running as a docker compose service


Once in the cli context you can launch scenarios with roslaunch:
```bash
cd adore_scenarios 
roslaunch demo001_loadmap.launch
```

### Building ADORe
Each module provides a Makefile and docker context for build. You can build the 
whole project by navigating to the ADORe project root and running:
```bash
cd adore
make build
```

### Static checking
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
