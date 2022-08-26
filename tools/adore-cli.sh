#!/usr/bin/env bash

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]:-$0}"; )" &> /dev/null && pwd 2> /dev/null; )";

#clear
bash tools/adore-cli_motd.sh
bash plotlabserver/tools/wait_for_plotlab_server.sh

echo ""
printf "  Waiting for catkin workspace ..."
until [ -e catkin_workspace/install/setup.sh ]; do
    printf "."
    sleep 1
done
printf " done \n"
source catkin_workspace/install/setup.sh

zsh
