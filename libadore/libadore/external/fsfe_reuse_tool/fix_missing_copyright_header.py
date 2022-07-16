#!/usr/bin/python3

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



import os

os.system("reuse lint > missing_licenses_info.txt")

input = open("missing_licenses_info.txt","r")

extensions = set()
fixed_files = []
unfixed_files = []

for line in input:
    # print(line)
    if line.startswith("* "):
        if line.find(":") == -1:
            line=line.strip()[2:]
            filename, file_ext = os.path.splitext(line)
            extensions.add(file_ext)
            style = ""
            if file_ext in [".h",".cpp",".c"]:
                style="c"
            elif file_ext in [".bash",".txt",".py",".config"]:
                style="python"
            elif file_ext in [".md"]:
                style="html"
            else:
                unfixed_files.append(line)
                continue
            os.system('reuse addheader -y 2019 -c "German Aerospace Center (DLR)" --license EPL-2.0 --style ' + style + " " + line)
            fixed_files.append(line)
print("Files that were fixed:")
for item in fixed_files:
    print("\t" + item)
print("Files that were not fixed (unknown extension or other issues):")
for item in unfixed_files:
    print("\t" + item)