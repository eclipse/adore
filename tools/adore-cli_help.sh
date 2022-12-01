#!/usr/bin/env bash

set -euo pipefail

echoerr (){ printf "%s" "$@" >&2;}
exiterr (){ echoerr "$@"; exit 1;}

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

printf "ADORe CLI Help\n"
printf "  At any time you can run 'make help' to get a list of available make targets.\n"
printf "\n"
printf "  Environment: \n"
printf "    ADORE_SOURCE_DIRECTORY: ${ADORE_SOURCE_DIRECTORY}\n"
printf "    CATKIN_WORKSPACE_DIRECTORY: $(realpath "${CATKIN_WORKSPACE_DIRECTORY}")\n"
printf "    ROS_ROOT: ${ROS_ROOT}\n"
printf "    ROS_DISTRO: ${ROS_DISTRO}\n"
printf "    ROS_MASTER_URI: ${ROS_MASTER_URI}\n"

printf "\n"

printf "  Getting Started:\n\n"
printf "    To execute a scenario run the following commands: \n"
printf "      cd adore_if_ros_demos \n"
printf "      roslaunch <scenario name> example: roslaunch baseline_test.launch\n\n"

printf "    To attach another termanal session to the ADORE cli at any time on the host you can run: \n"
printf "      'make adore-cli' or 'make cli' \n\n"

printf "    To exit the adore cli type 'exit' \n\n"

printf "    To stop the adore cli docker context type 'make stop_adore-cli' \n\n"


printf "%b" "\e[1;31mWARNING: All file system changes made OUTSIDE of the adore source directory will NOT persist!\e[0m\n\n"
