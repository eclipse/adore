#!/usr/bin/env bash

set -e

#CMAKE_PREFIX_PATH="$(realpath $(find -L . -name install | grep -e "install/\|CPack\|external" -v ))"
#echo "${CMAKE_PREFIX_PATH}"

CMAKE_PREFIX_PATH="$(realpath $(find -L . -name install) | tr "\n" ";")"
export CMAKE_PREFIX_PATH

cd /tmp/${PROJECT}/${PROJECT}/build

source /opt/ros/noetic/setup.bash
cmake .. 
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="install"
        -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
cmake --build . -v --config Release --target install -- -j $(nproc)
