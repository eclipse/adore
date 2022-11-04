#!/usr/bin/env bash

printf "Welcome to the ADORe Development CLI %s (%s %s %s)\n" "$(lsb_release -a 2>/dev/null | grep Description | cut -d: -f2 | xargs)" "$(uname -o)" "$(uname -r)" "$(uname -m)"
printf "  whoami: %s\n" "$(whoami)"
printf "  pwd: %s\n" "$(pwd)"
printf "  Catkin workspace directory: %s\n" "${CATKIN_WORKSPACE_DIRECTORY}"


printf "\n"
printf "  Getting Started:  \n"
printf "    To run tests use: make test \n"
printf "    Genearate a catkin workspace with: make create_catkin_workspace \n"
printf "    running scenarios: \n"
printf "      cd adore_if_ros_demos && roslaunch <launchfile> \n"
echo ""

