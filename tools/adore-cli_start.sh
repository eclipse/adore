#!/usr/bin/env bash

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]:-$0}"; )" &> /dev/null && pwd 2> /dev/null; )";


cd ${SCRIPT_DIR}/.. && make create_catkin_workspace

trap : TERM INT; sleep infinity & wait
