#!/usr/bin/env bash

# This script act as the main entrypoint for the adore-cli docker context.

set -euo pipefail

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echoerr (){ printf "%s" "$@" >&2;}
exiterr (){ echoerr "$@"; exit 1;}

ADORE_SOURCE_DIRECTORY=$(realpath "${SCRIPT_DIRECTORY}/..")

clear

cd "${ADORE_SOURCE_DIRECTORY}"

bash tools/adore-cli_motd.sh
bash plotlabserver/tools/wait_for_plotlab_server.sh

printf "\n"

export CATKIN_SHELL=sh

source catkin_workspace/install/setup.sh
zsh
