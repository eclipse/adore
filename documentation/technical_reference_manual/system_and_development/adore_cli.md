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
********************************************************************************
-->
## ADORe command line interface (CLI)

The ADORe CLI is a docker runtime context that provides a complete set of tools
for execution and development within adore. For more information on this tool
please visit https://github.com/DLR-TS/adore_cli

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
* All ROS tools preinstalled
* some basic development and debugging tools
* ZSH

### ADORe CLI Usage
Change directory to the root of the ADORe project and run:
```
make cli
```
On first run of the ADORe CLI the system will be built including all core
modules. Initial build can take 10-15 minutes depending on system and network. 

Once the ADORe CLI context builds and starts you will be presented with a 
zsh shell context:
```text
Welcome to the ADORe Development CLI Ubuntu 20.04.6 LTS (GNU/Linux 5.19.0-45-generic x86_64)

            ____ 
         __/  |_\__
        |           -. 
  ......'-(_)---(_)--' 

  Type 'help' for more information.

  Waiting for plotlab server ... plotlab server ready 

  Vehicle environment set to: Development

ADORe CLI: adore git:(main)  (0)>  
```
> **⚠ WARNING:** Any changes to the adore cli context require manually invoking a build for it to take effect! 
>The ADORE CLI is only built on first invocation of 'make cli'


This will build all necessary ADORe components and launch a docker context.


#### How do I know if I am in the ADORe CLI context?

- If you are in the ADORe CLI context you should have a shell prompt similar to
  the following: `ADORe CLI: adore git:(master)  (0)>`
- you can also check your current user with:
```bash
whoami
```
should report:
```bash
adore-cli
```

