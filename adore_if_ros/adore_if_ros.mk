# This Makefile contains useful targets that can be included in downstream projects.

ifndef ADORE_IF_ROS_MAKEFILE_PATH

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
ADORE_IF_ROS_PROJECT:=adore_if_ros

ADORE_IF_ROS_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
MAKE_GADGETS_PATH:=${ADORE_IF_ROS_MAKEFILE_PATH}/adore_if_ros_msg/make_gadgets
REPO_DIRECTORY:=${ADORE_IF_ROS_MAKEFILE_PATH}

ADORE_IF_ROS_TAG:=$(shell cd "${MAKE_GADGETS_PATH}" && make get_sanitized_branch_name REPO_DIRECTORY="${REPO_DIRECTORY}")
ADORE_IF_ROS_IMAGE:=${ADORE_IF_ROS_PROJECT}:${ADORE_IF_ROS_TAG}

ADORE_IF_ROS_CMAKE_BUILD_PATH:="${ADORE_IF_ROS_PROJECT}/build"
ADORE_IF_ROS_CMAKE_INSTALL_PATH:="${ADORE_IF_ROS_CMAKE_BUILD_PATH}/install"


.PHONY: build_adore_if_ros 
build_adore_if_ros: ## Build adore_if_ros
	cd "${ADORE_IF_ROS_MAKEFILE_PATH}" && make

.PHONY: test_adore_if_ros 
test_adore_if_ros: ## run adore_if_ros unit tests 
	cd "${ADORE_IF_ROS_MAKEFILE_PATH}" && make test

.PHONY: clean_adore_if_ros
clean_adore_if_ros: ## Clean adore_if_ros build artifacts
	cd "${ADORE_IF_ROS_MAKEFILE_PATH}" && make clean

.PHONY: branch_adore_if_ros
branch_adore_if_ros: ## Returns the current docker safe/sanitized branch for adore_if_ros
	@printf "%s\n" ${ADORE_IF_ROS_TAG}

.PHONY: image_adore_if_ros
image_adore_if_ros: ## Returns the current docker image name for adore_if_ros
	@printf "%s\n" ${ADORE_IF_ROS_IMAGE}

.PHONY: update_adore_if_ros
update_adore_if_ros:
	cd "${ADORE_IF_ROS_MAKEFILE_PATH}" && git pull
endif
