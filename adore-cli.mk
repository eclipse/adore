
# This Makefile contains useful targets that can be included in downstream projects.

ifndef ADORE-CLI_MAKEFILE_PATH

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
ADORE-CLI_PROJECT:=adore-cli

ADORE-CLI_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
MAKE_GADGETS_PATH:=${ADORE-CLI_MAKEFILE_PATH}/adore_if_ros_msg/make_gadgets
REPO_DIRECTORY:=${ADORE-CLI_MAKEFILE_PATH}

ADORE-CLI_TAG:=$(shell cd "${MAKE_GADGETS_PATH}" && make get_sanitized_branch_name REPO_DIRECTORY="${REPO_DIRECTORY}")
ADORE-CLI_IMAGE:=${ADORE-CLI_PROJECT}:${ADORE-CLI_TAG}

ADORE-CLI_CMAKE_BUILD_PATH:="${ADORE-CLI_PROJECT}/build"
ADORE-CLI_CMAKE_INSTALL_PATH:="${ADORE-CLI_CMAKE_BUILD_PATH}/install"

UID := $(shell id -u)
GID := $(shell id -g)

include adore_if_ros_msg/make_gadgets/docker/docker-tools.mk
include plotlabserver/plotlabserver.mk
include adore_if_ros/adore_if_ros.mk

.PHONY: build_adore-cli_fast
build_adore-cli_fast: # build adore-cli if it does not already exist in the docker repository. If it does exist this is a noop.
	@[ -n "$$(docker images -q ${ADORE-CLI_PROJECT}:${ADORE-CLI_TAG})" ] || \
    make build_adore-cli 

.PHONY: build_adore-cli
build_adore-cli: build_catkin_base ## Builds the ADORe CLI docker context/image
	cd plotlabserver build_fast_plotlabserver
	cd "${ADORE-CLI_MAKEFILE_PATH}" && \
    docker compose build ${ADORE-CLI_PROJECT} \
                         --build-arg UID=${UID} \
                         --build-arg GID=${GID} \
                         --build-arg DOCKER_GID=${DOCKER_GID} \
                         --build-arg ADORE_IF_ROS_TAG=${ADORE_IF_ROS_TAG} && \
    docker compose build adore-cli-x11-display \
                         --build-arg UID=${UID} \
                         --build-arg GID=${GID} \
                         --build-arg DOCKER_GID=${DOCKER_GID} \
                         --build-arg ADORE_IF_ROS_TAG=${ADORE_IF_ROS_TAG}

.PHONY: run_ci_scenarios
run_ci_scenarios:
	bash tools/run_ci_scenarios.txt 

.PHONY: adore-cli_setup
adore-cli_setup: build_adore-cli_fast
	@echo "Running adore-cli setup..."
	@mkdir -p .log/.ros/bag_files
	@mkdir -p plotlabserver/.log
	@cd .log && ln -sf ../plotlabserver/.log plotlabserver
	@touch .zsh_history
	@touch .zsh_history.new
	cd plotlabserver && \
    make down

.PHONY: adore-cli_teardown
adore-cli_teardown:
	@echo "Running adore-cli teardown..."
	@docker compose down && xhost - 1> /dev/null
	@docker compose rm -f

.PHONY: adore-cli_start
adore-cli_start:
	@xhost + && \
    docker compose up adore-cli-x11-display --force-recreate -V -d; \
    xhost - 

.PHONY: adore-cli_start_headless
adore-cli_start_headless:
	DISPLAY_MODE=headless make adore-cli_start

.PHONY: adore-cli_attach
adore-cli_attach:
	docker exec -it --user adore-cli adore-cli /bin/zsh -c "bash tools/adore-cli.sh" || true

.PHONY: adore-cli_scenarios_run
adore-cli_scenarios_run:
	docker exec -it --user adore-cli adore-cli /bin/zsh -c "bash tools/run_test_scenarios.sh" || true

.PHONY: adore-cli
adore-cli: adore-cli_setup adore-cli_start adore-cli_attach adore-cli_teardown ## Start an adore-cli context

endif
