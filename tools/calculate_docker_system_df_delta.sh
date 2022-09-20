#!/usr/bin/env bash

function echoerr { echo "$@" >&2; exit 1;}
SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd "${SCRIPT_DIRECTORY}/../.log"

prebuild_storage=$(cat docker_storage_inventory_prebuild.json | cut -d'"' -f 12 | grep -v -E "\[|\]|kB" | sed "s|GB||g" | awk '{s+=$1} END {print s}')
postbuild_storage=$(cat docker_storage_inventory_postbuild.json | cut -d'"' -f 12 | grep -v -E "\[|\]|kB" | sed "s|GB||g" | awk '{s+=$1} END {print s}')


echo $(( postbuild_storage - prebuild_storage ))"GB"


