#!/bin/bash
cwd=$(pwd)
shortcwd="${cwd##*/}"
echo "current folder: ${shortcwd}"
if [[ "${shortcwd}" == "tools" ]]; 
then
    prefix="../"
else
    prefix=""
fi
folders=(\
            "adore_if_carla" \
            "adore_if_ros" \
            "adore_if_v2x" \
            "adore_if_v2x" \
            "adore_v2x_sim" \
            "libadore" \
            "plotlabserver" \
            "plotlabserver/plotlablib" \
            "sumo_if_ros" \
            "v2x_if_ros_msg"\
        )
sudo apt-get update
for folder in ${folders[@]}; 
do
    dir="${prefix}${folder}/files"
    #echo "${dir}"
    for file in $dir/*; 
    do 
        if [ -f $file ] && [[ $file == *"ubuntu20.04.system"* ]]; 
        then
            echo "------------------------"
            echo "${file}:"
            cat ${file}
            echo "------------------------"
            sudo apt-get install --no-install-recommends -y $(sed '/^#/d' ${file} | sed '/^$/d')
        fi
    done
done