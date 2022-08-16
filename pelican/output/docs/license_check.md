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
*   Daniel Heß
********************************************************************************
-->
# How to check licenses
Ubuntu comes with a license checker:
~~~bash
 sudo apt-get install licensecheck
~~~
Before checking license, remove unnecessary files for release
~~~bash
./tools/release_purge.sh
~~~
Check for files, which do not contain EPL-2.0: 
Run the following in adore folder.
~~~bash
licensecheck . -r|egrep -ve ".png|.gif|.r2s|.jpg|build|sumo|java_v2x|.cmake|CMakeFiles|.gitlab-ci.yml|.vscode|Eclipse Public License 2.0"
~~~