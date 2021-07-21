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
# *  Thomas Lobig
# ********************************************************************************



echo "checking pip3 version"
pip3 --version

if [ $? -eq 0 ] ; then
    echo "pip3 found"
else
    echo "Please install pip3 first to continue"
    exit 1
fi

pip3 list --format=columns | tail -n +1 | grep -q "fsfe-reuse"

if [ $? -eq 0 ] ; then
    echo "fsfe-reuse tool is installed"
else
    echo "installing fsfe-reuse tool with pip3"
    pip3 install --user fsfe-reuse
fi



echo $PATH | grep -q .local/bin

if [ $? -eq 0 ] ; then
    echo ".local/bin is in path, GOOD!"
else
    echo "adding .local/bin to path"
    PATH=$PATH:$HOME/.local/bin
    export PATH
fi

echo "reuse is ready to be used"