#!/bin/bash

# SPDX-FileCopyrightText: 2019 German Aerospace Center (DLR)
#
# SPDX-License-Identifier: EPL-2.0

CONFIGFILE=adore_docs.config
if test -f "$CONFIGFILE"; then
    echo "Doxygen config found, starting the process"
    mkdir -p ../../install/docs_html
    mkdir -p ../../build/docs_temp
    pushd ../../build/docs_temp
    doxygen ../../src/documentation/$CONFIGFILE
    popd
    rm -r ../../build/docs_temp
    echo "Doxygenoutput can now be found in ../install/docs_html"
else
    echo "$CONFIGFILE not found, please execute this script from the folder which includes the config file."
fi