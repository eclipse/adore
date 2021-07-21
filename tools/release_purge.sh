#!/bin/bash
#********************************************************************************
#* Copyright (C) 2017-2020 German Aerospace Center (DLR). 
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
#*
#* This program and the accompanying materials are made available under the 
#* terms of the Eclipse Public License 2.0 which is available at
#* http://www.eclipse.org/legal/epl-2.0.
#*
#* SPDX-License-Identifier: EPL-2.0 
#*
#* Contributors: 
#*   Daniel Heß
#********************************************************************************
rm -rf v2x_if_ros/documentation #contains IP adresses
rm -rf v2x_if_ros/java_v2x #will be published separately
rm -f .gitlab-ci.yml #not relevant for github
rm -f documentation/gitlab_guide.md #not relevant for github
rm -rf .vscode
rm -rf sumo #downloaded automatically
rm -rf .git
rm -rf plotlab/server/build 
rm -rf plotlab/server/plotlablib
rm -rf plotlab/server/cmake
rm -f plotlab/server/include/plotlabserver/stb_image.h #downloaded automatically
