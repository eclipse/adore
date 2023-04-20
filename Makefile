SHELL:=/bin/bash

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")

#include adore_if_ros/adore_if_ros.mk
include make_gadgets/make_gadgets.mk
#include catkin_docker/catkin_base.mk

.EXPORT_ALL_VARIABLES:
CATKIN_WORKSPACE_DIRECTORY:=${ROOT_DIR}/catkin_workspace
SOURCE_DIRECTORY:=${ROOT_DIR}
ADORE_CLI_WORKING_DIRECTORY:=${ROOT_DIR}
SUBMODULES_PATH:=${ROOT_DIR}

include adore_cli/adore_cli.mk


.PHONY: test
test:
	xhost + && \
    docker compose -f ${DOCKER_COMPOSE_FILE} up adore-cli_x11-display \
      --force-recreate \
      --renew-anon-volumes \
      --detach; \
    xhost - 
