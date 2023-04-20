SHELL:=/bin/bash

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")

include make_gadgets/make_gadgets.mk

.EXPORT_ALL_VARIABLES:
CATKIN_WORKSPACE_DIRECTORY:=${ROOT_DIR}/catkin_workspace
SOURCE_DIRECTORY:=${ROOT_DIR}
ADORE_CLI_WORKING_DIRECTORY:=${ROOT_DIR}
SUBMODULES_PATH:=${ROOT_DIR}

include adore_cli/adore_cli.mk

