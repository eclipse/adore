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
#!/bin/bash
# parameter $1: filename(s) of bag files
# parameter $2: option for rostopic echo format: e.g. -p for csv output
dirname="${1%% **}_txt" 
mkdir -p ${dirname}
rostopic list -b $1>"${dirname}/topics.txt"
while read line; do
  filename=${line//[\/]/_}; #replace "/"" with "_"
  rostopic echo $2 ${line} > "${dirname}/${filename}.txt" &
done <"${dirname}/topics.txt"
sleep 1;
rosbag play -q -i --clock --hz=100 $1;
sleep 1;
killall rostopic;