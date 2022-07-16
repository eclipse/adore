#!/usr/bin/env bash

set -e

function echoerr { echo "$@" >&2; exit 1;}
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )" 

ADORE_SOURCE_DIRECTORY=$(realpath "${DIR}/..")

if ! [ -x "$(command -v catkin)" ]; then
    echoerr "ERROR: catkin not installed."
fi

if [[ -z ${CATKIN_WORKSPACE_DIRECTORY+x} ]]; then
    CATKIN_WORKSPACE_DIRECTORY="catkin_workspace"
fi

if [[ ! -d "${CATKIN_WORKSPACE_DIRECTORY}" ]]; then
    mkdir "${CATKIN_WORKSPACE_DIRECTORY}"
    mkdir "${CATKIN_WORKSPACE_DIRECTORY}"/{build,devel,install,logs,src}
    mkdir -p "${CATKIN_WORKSPACE_DIRECTORY}"/install/{lib/python3/dist-packages,share,include}
	
    cd $CATKIN_WORKSPACE_DIRECTORY
    for file in $ADORE_SOURCE_DIRECTORY/*; do 
    	if [ -d "$file" ]; then
	    	#echo "processing: $file"
	    	#short="${${file:0:-1}##*/}"    #extract dir name without path and trailing /
	    	short="${file##*/}"    #extract dir name without path 
	    	#echo "subfolder: $short"
	    	#echo "processing: $file/$short"
		if [[ ! "$file" =~ ${CATKIN_WORKSPACE_DIRECTORY} ]]; then 
		    	if [ -d "$file/$short" ]; then  #if directory contains subdirectory of same name
			    if [ -f "$file/$short/package.xml" ]; then #if subdirectory contanis a package.xml
			    	    #then create link to subdirectory in src
				    ln -s "$file/$short" "src/"  
				    echo "creating link src/$short -> $file/$short"
				    #then create link to lib
				    if [ -d "$file/$short/build/install/lib/$short" ]; then 
					    ln -s "$file/$short/build/install/lib/$short" "install/lib/$short"  
					    echo "creating link install/lib/$short -> $file/$short/build/install/lib/$short"
					#alternatively use devel
				    elif [ -d "$file/$short/build/devel/lib/$short" ]; then 
					    ln -s "$file/$short/build/devel/lib/$short" "install/lib/$short"  
					    echo "creating link install/lib/$short -> $file/$short/build/devel/lib/$short"
				    fi
				    #create folder install/share/package-name
				    mkdir -p "install/share/$short"
				    #then create link to share/cmake
				    ln -s "$file/$short/build/install/share/$short/cmake" "install/share/$short/cmake"  
				    echo "creating link install/share/$short/cmake -> $file/$short/build/install/share/$short/cmake"
				    #then create link to package.xml
				    ln -s "$file/$short/package.xml" "install/share/$short/package.xml"  
				    echo "creating link install/share/$short/package.xml -> $file/$short/package.xml"
				    #then create link to msg folder
				    if [ -d "$file/$short/msg" ]; then
				    	ln -s "$file/$short/msg" "install/share/$short/msg"  
				    	echo "creating link install/share/$short/msg -> $file/$short/msg"
				    fi
				    #then create link to include/package-name
				    if [ -d "$file/$short/include" ]; then
				    	ln -s "$file/$short/include/$short" "install/include/$short"  
				    	echo "creating link install/include/$short -> $file/$short/include/$short"
				    fi
				    #python3 content generated for ros messages
				    if [ -d "$file/$short/build/install/lib/python3/dist-packages/$short" ]; then
				    	ln -s "$file/$short/build/install/lib/python3/dist-packages/$short" "install/lib/python3/dist-packages/$short"  
				    	echo "creating link install/lib/python3/dist-packages/$short  -> $file/$short/build/install/lib/python3/dist-packages/$short"
				    fi
			    fi
		            #ln -s "$file/$short/build/install/share/$short" "install/share/$short"  
			    #echo "creating link install/share/$short -> $file/$short/build/install/share/$short"
			elif [ "$short" == "tools" ]; then
			    ln -s "$file" "src/"  #create tools folder
			    echo "creating link src/$short -> $file"
			elif [ "$short" == "adore_if_ros_demos" ]; then
			    ln -s "$file" "src/"  #create demos folder
			    echo "creating link src/$short -> $file"
			fi
		fi 
	fi
    done
	# add a link to the sumo lib, if it exists
	if [ -f "$ADORE_SOURCE_DIRECTORY/sumo_if_ros/sumo/build/install/lib/libsumocpp.so" ]; then
		ln -s "$ADORE_SOURCE_DIRECTORY/sumo_if_ros/sumo/build/install/lib/libsumocpp.so" "install/lib/libsumocpp.so"
		echo "install/lib/libsumocpp.so -> $ADORE_SOURCE_DIRECTORY/sumo_if_ros/sumo/build/install/lib/libsumocpp.so"
	fi
    source /opt/ros/noetic/setup.bash
    catkin config --init --install --extend /opt/ros/noetic/
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_adore_TESTING=ON -DXSD_INCLUDE_DIR=include -DCMAKE_PREFIX_PATH=/opt/ros/noetic
    (cd src
    catkin_create_pkg catkin_workspace_init
    )
    catkin build catkin_workspace_init
    rm -rf src/catkin_workspace_init
else
    echoerr "ERROR: The Catkin workspace directory: ${CATKIN_WORKSPACE_DIRECTORY} already exists."
fi
echo "The Catkin workspace directory can be found at: " $(realpath .)
