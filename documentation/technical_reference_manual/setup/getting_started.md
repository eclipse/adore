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

1. First review the [System Requirements 🔗](system_requirements.md). 

2. Next review the [Prerequisites 🔗](prerequisites.md) 

3. Clone the repository: 

    ```bash
    git clone git@github.com:eclipse/adore.git
    ```

4. Initialize and update the submodules:

    ```bash
    cd adore
    git submodule update --init
    ```

    > **⚠️ WARNING:**
    > Failing to update the submodules will result in build failures!

5. Build ADORe and ADORe CLI:

> **ℹ️ INFO:**
> It is recommended doing a first build of ADORe connected to hard line
> ethernet and **not** WIFI until initial docker and APT caches are established
> on your system.

After cloning ADORe and satisfying all system prerequisites you can build/run
the ADORe CLI context. To do this navigate to the root of the ADORe directory
and run the following command:
```bash
make cli
```

`make cli` on first run will build/compile all of the core ADORe components and
build the ADORe cli docker context. If you wish to run other non-core components
you want to run you will need to invoke `make build_all` or build them manually. 

For more informant please visit the [ADORe CLI 🔗](../system_and_development/adore_cli.md) documentation.

> **✅ SUCCESS:**
> If you are greeted with the following ADORe CLI car then you have successfully setup ADORe:
```
            ____ 
         __/  |_\__
        |           -. 
  ......'-(_)---(_)--' 
```

> **ℹ️ INFO:**
> By default the setup script clones the master branch at: [https://github.com/eclipse/adore.git 🔗](https://github.com/eclipse/adore.git). 
> You may need to fork, change branch, add remotes, or setup ssh keys.

Once you have successfully built AOREe you can take a look at:
