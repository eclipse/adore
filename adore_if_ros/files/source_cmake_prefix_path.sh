#!/usr/bin/env bash

CMAKE_PREFIX_PATH="$(realpath $(find -L . -name install) | tr "\n" ";")"
export CMAKE_PREFIX_PATH
