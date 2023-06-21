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

1. First review the system requirements:
   [system_requirements.md](system_requirements.md) 

2. Next review the prerequisites:[prerequisites](prerequisites.md) 

3. Clone the repository: 
```bash
git clone git@github.com:eclipse/adore.git
```

4. Initialize and update the submodules:
```bash
cd adore
git submodule update --init
```
> :warning: **Warning:** Failing to update the submodules will result in build failures!

5. Build ADORe and ADORe CLI
After cloning ADORe and satifisfying all system prerequisites you can build/run
the ADORe CLI context. To do this navigate to the root of the ADORe directory
and run the following command:
```bash
make cli
```
For more informant on the ADORe CLI visit:
[system_and_development/adore_cli.md]







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
By default only core modules are built to save time. To build all ADORe modules you can run the provided target:
```bash
make build_all
```

