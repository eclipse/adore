#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echoerr (){ printf "%s" "$@" >&2;}
exiterr (){ echoerr "$@"; exit 1;}

clear

cd "${SCRIPT_DIRECTORY}"/..

make create_catkin_workspace > .log/create_catkin_workspace.log 2>&1 &

bash tools/adore-cli_motd.sh
bash plotlabserver/tools/wait_for_plotlab_server.sh

printf "\n"
printf "  Waiting for catkin workspace ..."
until [ -e catkin_workspace/install/setup.sh ]; do
    printf "."
    sleep 1
done
printf " done \n"
source catkin_workspace/install/setup.sh

zsh
