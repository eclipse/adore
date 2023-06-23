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
## Getting Started
This guide will help you get your system set up and configure to run ADORe.

## Getting started with ADORe

1. First review the system requirements: [system_requirements.md](system_requirements.md) 

2. Next review the prerequisites: [prerequisites.md](prerequisites.md) 

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

## Next
Whats next????

