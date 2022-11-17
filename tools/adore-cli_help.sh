#!/usr/bin/env bash

set -euo pipefail

echoerr (){ printf "%s" "$@" >&2;}
exiterr (){ echoerr "$@"; exit 1;}

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

printf "ADORe CLI Help\n"
printf "  At any time you can run 'make help' to get a list of available make targets.\n"
printf "\n"
printf "  Environment: \n"
printf "    ADORE_SOURCE_DIR: ${ADORE_SOURCE_DIR}\n"
printf "    ROS_ROOT: ${ROS_ROOT}\n"
printf "    ROS_DISTRO: ${ROS_DISTRO}\n"
printf "    ROS_MASTER_URI: ${ROS_MASTER_URI}\n"
