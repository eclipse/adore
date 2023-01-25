
ifndef ADORE_MAKEFILE_PATH

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:

ADORE_PROJECT=adore

ADORE_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
MAKE_GADGETS_PATH:=${ADORE_MAKEFILE_PATH}/adore_if_ros_msg/make_gadgets
REPO_DIRECTORY:=${ADORE_MAKEFILE_PATH}

include ${ADORE_MAKEFILE_PATH}/adore-cli.mk
include ${ADORE_CLI_MAKEFILE_PATH}/adore_if_ros/adore_if_ros.mk

.PHONY: build_fast_adore_if_ros
build_fast_adore_if_ros: # Build adore_if_ros if it does not already exist in the docker repository. If it does exist this is a noop.
	@if [ -n "$$(docker images -q ${ADORE_IF_ROS_PROJECT}:${ADORE_IF_ROS_TAG})" ]; then \
        echo "Docker image: ${ADORE_IF_ROS_PROJECT}:${ADORE_IF_ROS_TAG} already build, skipping build."; \
    else \
        cd "${ADORE_MAKEFILE_PATH}" && ${CLEAR_ENV} make build;\
    fi

endif
