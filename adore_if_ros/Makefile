SHELL:=/bin/bash

.DEFAULT_GOAL := all

.PHONY: all 
all: root_check docker_group_check build

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")
MAKEFILE_PATH:=$(shell dirname "$(abspath "$(lastword $(MAKEFILE_LIST)"))")

MAKEFLAGS += --no-print-directory

include adore_if_ros_msg/make_gadgets/make_gadgets.mk
include adore_if_ros_msg/make_gadgets/docker/docker-tools.mk
include cppcheck_docker/cppcheck_docker.mk
include cpplint_docker/cpplint_docker.mk
include lizard_docker/lizard_docker.mk
include plotlablib/plotlablib.mk 
include ../libadore/libadore.mk
include adore_scheduling/adore_scheduling.mk


.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

CPP_PROJECT_DIRECTORY:="${ROOT_DIR}/adore_if_ros"

include adore_if_ros.mk 
include adore_if_ros_msg/adore_if_ros_msg.mk 

#CMAKE_PREFIX_PATH?=$(shell realpath "$$(find . -name install | grep "install" | grep -e "install/\|CPack" -v)" | tr '\n' ';') 
CMAKE_PREFIX_PATH?=$(shell realpath "$$(find . -type d | grep "build" | grep -v "build/")" 2>/dev/null | tr '\n' ';')

.PHONY: set_env 
set_env: 
	$(eval PROJECT := ${ADORE_IF_ROS_PROJECT}) 
	$(eval TAG := ${ADORE_IF_ROS_TAG})

.PHONY: build 
build: clean set_env build_libadore build_plotlablib build_adore_scheduling## Build adore_if_ros
	unset ADORE_IF_ROS_MSG_MAKEFILE_PATH && \
    make --file=${ADORE_IF_ROS_MAKEFILE_PATH}/adore_if_ros_msg/adore_if_ros_msg.mk build_adore_if_ros_msg 
	rm -rf "${PROJECT}/build"
	docker build --tag ${PROJECT}:${TAG} \
                 --network host \
                 --build-arg ADORE_IF_ROS_MSG_TAG="${ADORE_IF_ROS_MSG_TAG}" \
                 --build-arg LIBADORE_TAG="${LIBADORE_TAG}" \
				 --build-arg ADORE_IF_ROS_SCHEDULING_TAG=${ADORE_IF_ROS_SCHEDULING_TAG} \
                 --build-arg PLOTLABLIB_TAG="${PLOTLABLIB_TAG}" .
	docker cp $$(docker create --rm ${PROJECT}:${TAG}):/tmp/${PROJECT}/${PROJECT}/build ${PROJECT}

.PHONY: clean_submodules
clean_submodules:
	unset ADORE_IF_ROS_MSG_MAKEFILE_PATH && \
    make --file=${ADORE_IF_ROS_MAKEFILE_PATH}/adore_if_ros_msg/adore_if_ros_msg.mk clean_adore_if_ros_msg 
	unset PLOTLABLIB_MAKEFILE_PATH && \
    make --file=${ADORE_IF_ROS_MAKEFILE_PATH}/plotlablib/plotlablib.mk clean_plotlablib 

.PHONY: clean 
clean: set_env clean_submodules
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	find . -name "**lint_report.log" -exec rm -rf {} \;
	find . -name "**cppcheck_report.log" -exec rm -rf {} \;
	find . -name "**lizard_report.xml" -exec rm -rf {} \;
	docker rm $$(docker ps -a -q --filter "ancestor=${PROJECT}:${TAG}") --force 2> /dev/null || true
	docker rmi $$(docker images -q ${PROJECT}:${TAG}) --force 2> /dev/null || true
	docker rmi $$(docker images --filter "dangling=true" -q) --force > /dev/null 2>&1 || true

.PHONY: static_checks
static_checks: lint lizard cppcheck

