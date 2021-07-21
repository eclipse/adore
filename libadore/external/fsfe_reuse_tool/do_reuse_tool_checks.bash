#!/bin/bash

# *******************************************************************************
# * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
# * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# *
# * This program and the accompanying materials are made available under the 
# * terms of the Eclipse Public License 2.0 which is available at
# * http://www.eclipse.org/legal/epl-2.0.
# *
# * SPDX-License-Identifier: EPL-2.0 
# *
# * Contributors: 
# * Thomas Lobig
# ********************************************************************************


SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

echo "checking reuse tool version"
reuse --version

if [ $? -eq 0 ] ; then
    echo "reuse tool found, ready to do checks"
else
    echo "Please install fsfe-reuse first to continue"
    exit 1
fi

pushd $SCRIPTPATH/../.. > /dev/null
reuse init
reuse lint
popd