#!/usr/bin/env bash

set -euo pipefail
#set -euxo pipefail

echoerr (){ printf "%s" "$@" >&2;}
exiterr (){ echoerr "$@"; exit 1;}

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

ADORE_SOURCE_DIR="$(realpath "${SCRIPT_DIRECTORY}/..")"

cd "${ADORE_SOURCE_DIR}"
find . -name make_gadgets -type d | grep -v .git | while read line ; do
    (cd "${line}" && git checkout master && git pull)
done

find . -name cpplint_docker -type d | grep -v .git | while read line ; do
    (cd "${line}" && git checkout master && git pull)
done

find . -name cppcheck_docker -type d | grep -v .git | while read line ; do
    (cd "${line}" && git checkout master && git pull)
done

find . -name lizard_docker -type d | grep -v .git | while read line ; do
    (cd "${line}" && git checkout master && git pull)
done
