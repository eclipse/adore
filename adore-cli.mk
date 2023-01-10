
# This Makefile contains useful targets that can be included in downstream projects.

ifndef ADORE_CLI_MAKEFILE_PATH

MAKEFLAGS += --no-print-directory

CLEAR_ENV=env -i

.EXPORT_ALL_VARIABLES:
ADORE_CLI_PROJECT:=adore-cli

ADORE_CLI_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
MAKE_GADGETS_PATH:=${ADORE_CLI_MAKEFILE_PATH}/adore_if_ros_msg/make_gadgets
REPO_DIRECTORY:=${ADORE_CLI_MAKEFILE_PATH}

ADORE_CLI_TAG:=$(shell cd "${MAKE_GADGETS_PATH}" && make get_sanitized_branch_name REPO_DIRECTORY="${REPO_DIRECTORY}")
ADORE_CLI_IMAGE:=${ADORE_CLI_PROJECT}:${ADORE_CLI_TAG}

ADORE_CLI_CMAKE_BUILD_PATH:="${ADORE_CLI_PROJECT}/build"
ADORE_CLI_CMAKE_INSTALL_PATH:="${ADORE_CLI_CMAKE_BUILD_PATH}/install"

ADORE_SOURCE_DIRECTORY:=${REPO_DIRECTORY}

DOCKER_COMPOSE_FILE?=docker-compose.yaml

ADORE_CLI_WORKING_DIRECTORY?=${ADORE_SOURCE_DIRECTORY}

UID := $(shell id -u)
GID := $(shell id -g)


TEST_SCENARIOS?=baseline_test.launch

#include adore_if_ros_msg/make_gadgets/docker/docker-tools.mk
include ${ADORE_CLI_MAKEFILE_PATH}/apt_cacher_ng_docker/apt_cacher_ng_docker.mk 
include ${ADORE_CLI_MAKEFILE_PATH}/plotlabserver/plotlabserver.mk
include ${ADORE_CLI_MAKEFILE_PATH}/adore.mk
include ${ADORE_CLI_MAKEFILE_PATH}/catkin_base.mk

.PHONY: adore_if_ros_check
adore_if_ros_check:
	@if [ -z "$$(docker images -q '${ADORE_IF_ROS_PROJECT}:${ADORE_IF_ROS_TAG}')" ]; then \
        echo "adore_if_ros docker image: ${ADORE_IF_ROS_PROJECT}:${ADORE_IF_ROS_TAG} does not exits in the local docker repository. "; \
        echo "Did you build adore_if_ros?"; \
        echo "  Hint: run 'make build' to build adore_if_ros."; \
        exit 1; \
    fi

.PHONY: adore-cli_up
adore-cli_up: adore-cli_setup adore-cli_start adore-cli_attach adore-cli_teardown 

.PHONY: cli
cli: adore-cli ## Same as 'make adore-cli' for the lazy 

.PHONY: stop_adore-cli
stop_adore-cli: docker_host_context_check adore-cli_teardown ## Stop adore-cli docker context if it is running

.PHONY: adore-cli 
adore-cli: docker_host_context_check build_fast_adore_if_ros build_fast_adore-cli ## Start adore-cli context or attach to it if already running
	@if [[ "$$(docker inspect -f '{{.State.Running}}' '${ADORE_CLI_PROJECT}' 2>/dev/null)" == "true"  ]]; then\
        unset ADORE_CLI_MAKEFILE_PATH && make --file=${ADORE_CLI_MAKEFILE_PATH}/adore-cli.mk adore-cli_attach;\
        exit 0;\
    else\
        unset ADORE_CLI_MAKEFILE_PATH && make --file=${ADORE_CLI_MAKEFILE_PATH}/adore-cli.mk adore-cli_up;\
        exit 0;\
    fi;

.PHONY: build_fast_adore-cli
build_fast_adore-cli: # build the adore-cli conte does not already exist in the docker repository. If it does exist this is a noop.
	@if [ -n "$$(docker images -q ${ADORE_CLI_PROJECT}:${ADORE_CLI_TAG})" ]; then \
        echo "Docker image: ${ADORE_CLI_PROJECT}:${ADORE_CLI_TAG} already build, skipping build."; \
    else \
        unset ADORE_CLI_MAKEFILE_PATH && make --file=${ADORE_CLI_MAKEFILE_PATH}/adore-cli.mk build_adore-cli;\
    fi


.PHONY: build_adore-cli
build_adore-cli: clean_adore-cli ## Builds the ADORe CLI docker context/image
	cd ${APT_CACHER_NG_DOCKER_MAKEFILE_PATH} && make up
	cd ${ADORE_CLI_MAKEFILE_PATH}/plotlabserver && make build_fast_plotlabserver
	cd "${ADORE_CLI_MAKEFILE_PATH}" && \
    docker compose build ${ADORE_CLI_PROJECT} \
                         --build-arg UID=${UID} \
                         --build-arg GID=${GID} \
                         --build-arg DOCKER_GID=${DOCKER_GID} \
                         --build-arg ADORE_IF_ROS_TAG=${ADORE_IF_ROS_TAG} && \
    docker compose build adore-cli_x11-display \
                         --build-arg UID=${UID} \
                         --build-arg GID=${GID} \
                         --build-arg DOCKER_GID=${DOCKER_GID} \
                         --build-arg ADORE_CLI_TAG=${ADORE_CLI_TAG}

.PHONY: clean_adore-cli 
clean_adore-cli: ## Clean adore-cli docker context 
	docker rm $$(docker ps -a -q --filter "ancestor=${CATKIN_BASE_PROJECT}:${ADORE_CLI_TAG}") --force 2> /dev/null || true
	docker rm $$(docker ps -a -q --filter "ancestor=${CATKIN_BASE_PROJECT}_x11-display:${ADORE_CLI_TAG}") --force 2> /dev/null || true
	docker rmi $$(docker images -q ${ADORE_CLI_PROJECT}:${ADORE_CLI_TAG}) --force 2> /dev/null || true
	docker rmi $$(docker images -q ${ADORE_CLI_PROJECT}_x11-display:${ADORE_CLI_TAG}) --force 2> /dev/null || true
	docker rmi $$(docker images --filter "dangling=true" -q) --force > /dev/null 2>&1 || true

.PHONY: run_ci_scenarios
run_ci_scenarios:
	bash tools/run_ci_scenarios.sh 

.PHONY: run_test_scenarios
run_test_scenarios: adore-cli_setup adore-cli_start_headless adore-cli_scenarios_run adore-cli_teardown # run headless test scenarios

.PHONY: adore-cli_setup
adore-cli_setup: 
	@echo "Running adore-cli setup..."
	unset ADORE_CLI_MAKEFILE_PATH && make --file=${ADORE_CLI_MAKEFILE_PATH}/adore-cli.mk build_fast_adore-cli
	unset CATKIN_BASE_MAKEFILE_PATH && make --file=${CATKIN_BASE_MAKEFILE_PATH}/catkin_base.mk initialize_catkin_workspace
	@mkdir -p .log/.ros/bag_files
	@mkdir -p ${ADORE_CLI_MAKEFILE_PATH}/plotlabserver/.log
	@cd .log && ln -sf ../${ADORE_CLI_MAKEFILE_PATH}/plotlabserver/.log plotlabserver
	@touch .zsh_history
	@touch .zsh_history.new
	cd ${ADORE_CLI_MAKEFILE_PATH}/plotlabserver && \
    make down || true

.PHONY: adore-cli_teardown
adore-cli_teardown:
	@echo "Running adore-cli teardown..."
	@docker compose -f ${DOCKER_COMPOSE_FILE} down && xhost - 1> /dev/null || true
	@docker compose -f ${DOCKER_COMPOSE_FILE} rm -f

.PHONY: adore-cli_start
adore-cli_start:
	xhost + && \
    docker compose -f ${DOCKER_COMPOSE_FILE} up adore-cli_x11-display --force-recreate -V -d; \
    xhost - 

.PHONY: adore-cli_start_headless
adore-cli_start_headless:
	DISPLAY_MODE=headless unset ADORE_CLI_MAKEFILE_PATH && make --file=${ADORE_CLI_MAKEFILE_PATH}/adore-cli.mk adore-cli_start 

.PHONY: adore-cli_attach
adore-cli_attach:
	docker exec -it --user adore-cli adore-cli /bin/zsh -c "ADORE_CLI_WORKING_DIRECTORY=${ADORE_CLI_WORKING_DIRECTORY} bash /tmp/adore/tools/adore-cli.sh" || true

.PHONY: adore-cli_scenarios_run
adore-cli_scenarios_run:
	docker exec -it --user adore-cli adore-cli /bin/zsh -c "bash tools/run_test_scenarios.sh" || true

.PHONY: run_test_scenarios
run_test_scenarios: adore-cli_setup adore-cli_start_headless adore-cli_scenarios_run adore-cli_teardown ## Run adore test scenarios specified by the TEST_SCENARIOS environmental variable
	@echo "  To run alternative scenarios call 'make run_test_scenarios' by modifying the environmental variable TEST_SCENARIOS."
	@echo "    Usage examples: "
	@echo "      make run_test_scenarios TEST_SCENARIOS=baseline_test.launch"
	@echo "      make run_test_scenarios TEST_SCENARIOS=a.launch b.launch c.launch"
	@echo "      make run_test_scenarios DISPLAY_MODE=headless TEST_SCENARIOS=baseline_test.launch"
	@echo "      make run_test_scenarios DISPLAY_MODE=native TEST_SCENARIOS=baseline_test.launch"
	@echo "      make run_test_scenarios DISPLAY_MODE=window_manager TEST_SCENARIOS=baseline_test.launch"



endif
