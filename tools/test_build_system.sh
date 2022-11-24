#!/usr/bin/env bash

set -euo pipefail
#set -euxo pipefail

echoerr (){ printf "%s" "$@" >&2;}
exiterr (){ echoerr "$@"; exit 1;}

echodebug (){ 
    if [ ! -z ${DEBUG+x} ] && [ "DEBUG" -eq true ]; then 
        printf "$@\n" >&2;
    fi
}
SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ADORE_DIRECTORY="$(realpath "${SCRIPT_DIRECTORY}/..")"

cd "${ADORE_DIRECTORY}"

yes | make docker_orbital_cannon
make build
make test
make lizard
make lint | true
