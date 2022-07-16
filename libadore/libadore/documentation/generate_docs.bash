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
#*   Thomas Lobig
#********************************************************************************


CONFIGFILE=adore_docs.config
if test -f "$CONFIGFILE"; then
    echo "Doxygen config found, starting the process"
    mkdir -p ../build/docs_html
    mkdir -p ../build/docs_temp
    pushd ../build/docs_temp
    doxygen ../../documentation/$CONFIGFILE
    popd
    rm -r ../build/docs_temp
    echo "Doxygenoutput can now be found in ../build/docs_html"
else
    echo "$CONFIGFILE not found, please execute this script from the folder which includes the config file."
fi