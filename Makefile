include adore_if_ros/make_gadgets/Makefile
include adore_if_ros/make_gadgets/docker/Makefile

SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")
MAKEFLAGS += --no-print-directory


.EXPORT_ALL_VARIABLES:
CATKIN_WORKSPACE_DIRECTORY=catkin_workspace

DOCKER_BUILDKIT?=1
COMPOSE_DOCKER_CLI_BUILD?=1 
DOCKER_CONFIG?=$(shell realpath "${ROOT_DIR}")/apt_cacher_ng_docker

DOCKER_GID := $(shell getent group | grep docker | cut -d":" -f3)
USER := $(shell whoami)
UID := $(shell id -u)
GID := $(shell id -g)

TEST_SCENARIOS?=baseline_test.launch baseline_test.launch


.PHONY: all
all: \
     docker_group_check \
     root_check \
     start_apt_cacher_ng \
     build_adore_if_ros_msg\
     build_adore_v2x_sim \
     build_adore_if_v2x \
     build_sumo_if_ros \
     build_plotlabserver \
     build_libadore \
     build_adore_if_ros \
     get_apt_cacher_ng_cache_statistics \
     stop_apt_cacher_ng

.PHONY: build
build: all

.PHONY: clean
clean: delete_all_none_tags 
	cd plotlabserver && make clean
	cd sumo_if_ros && make clean
	cd adore_if_ros_msg && make clean
	cd libadore && make clean
	cd adore_if_ros && make clean
	cd adore_if_v2x && make clean

.PHONY: start_apt_cacher_ng 
start_apt_cacher_ng: ## Start apt cacher ng service
	cd apt_cacher_ng_docker && \
    make up

.PHONY: stop_apt_cacher_ng 
stop_apt_cacher_ng: ## Stop apt cacher ng service
	cd apt_cacher_ng_docker && make down

.PHONY: get_apt_cacher_ng_cache_statistics 
get_apt_cacher_ng_cache_statistics: ## returns the cache statistics for apt cahcer ng
	@cd apt_cacher_ng_docker && \
	make get_cache_statistics

.PHONY: submodules_update 
submodules_update: # Updates submodules
	git submodule update --init --recursive

.PHONY: build_adore_if_ros 
build_adore_if_ros: ## build adore_if_ros
	cd adore_if_ros && \
    make
.PHONY: build_adore_if_ros_msg
build_adore_if_ros_msg: 
	cd adore_if_ros_msg && \
	make
	
.PHONY: build_plotlabserver 
build_plotlabserver: ## Build plotlabserver
	cd plotlabserver && \
    make build_fast

.PHONY: build_adore_if_v2x 
build_adore_if_v2x: ## Build adore_if_v2x
	cd adore_if_v2x && \
    make

.PHONY: build_adore_v2x_sim 
build_adore_v2x_sim: ## Build adore_v2x_sim
	cd adore_v2x_sim && \
    make

.PHONY: build_libadore 
build_libadore: start_apt_cacher_ng ## Build libadore
	cd libadore && \
    make

.PHONY: build_sumo_if_ros 
build_sumo_if_ros: ## Build sumo_if_ros
	cd sumo_if_ros && \
    make

.PHONY: test 
test:
	mkdir -p .log && \
    cd libadore && \
	make test | tee "${ROOT_DIR}/.log/libadore_unit_test.log"; exit $$PIPESTATUS

.PHONY: lint_sumo_if_ros 
lint_sumo_if_ros:
	cd sumo_if_ros && make lint

.PHONY: lint 
lint: ## Run linting for all modules
	mkdir -p .log
	find . -name "**lint_report.log" -exec rm -rf {} \;
	EXIT_STATUS=0; \
        (cd sumo_if_ros && make lint) || EXIT_STATUS=$$? && \
        (cd libadore && make lint) || EXIT_STATUS=$$? && \
        (cd adore_if_ros && make lint) || EXIT_STATUS=$$? && \
	    find . -name "**lint_report.log" -print0 | xargs -0 -I {} mv {} .log/ && \
        exit $$EXIT_STATUS
 
.PHONY: lizard 
lizard: ## Run lizard static analysis tool for all modules
	mkdir -p .log
	find . -name "**lizard_report.**" -exec rm -rf {} \;
	EXIT_STATUS=0; \
        (cd sumo_if_ros && make lizard) || EXIT_STATUS=$$? && \
        (cd libadore && make lizard) || EXIT_STATUS=$$? \ && \
        (cd adore_if_ros && make lizard) || EXIT_STATUS=$$? && \
	    find . -name "**lizard_report.**" -print0 | xargs -0 -I {} mv {} .log/ && \
        exit $$EXIT_STATUS

.PHONY: cppcheck 
cppcheck: ## Run cppcheck static checking tool for all modules.
	mkdir -p .log
	find . -name "**cppcheck_report.log" -exec rm -rf {} \;
	EXIT_STATUS=0; \
        (cd sumo_if_ros && make cppcheck) || EXIT_STATUS=$$? && \
        (cd libadore && make cppcheck) || EXIT_STATUS=$$? && \
        (cd adore_if_ros && make cppcheck) || EXIT_STATUS=$$? && \
	    find . -name "**cppcheck_report.log" -print0 | xargs -0 -I {} mv {} .log/ && \
        exit $$EXIT_STATUS

.PHONY: clean_catkin_workspace 
clean_catkin_workspace:
	rm -rf "${CATKIN_WORKSPACE_DIRECTORY}"

.PHONY: build_catkin_base 
build_catkin_base: ## Build a docker image with base catkin tools installed with tag catkin_base:latest
	cd docker && \
    docker build \
                 --network host \
                 --build-arg UID=${UID} \
                 --build-arg GID=${GID} \
                 --file Dockerfile.catkin_base \
                 --tag catkin_base .

.PHONY: create_catkin_workspace_docker
create_catkin_workspace_docker: build_catkin_base
	docker run -it \
                   --user "${UID}:${GID}" \
                   --mount type=bind,source="${ROOT_DIR}",target="${ROOT_DIR}" \
                   catkin_base \
				   /bin/bash -c 'cd "${ROOT_DIR}" && HOME="${ROOT_DIR}" CATKIN_WORKSPACE_DIRECTORY="${CATKIN_WORKSPACE_DIRECTORY}" bash tools/create_catkin_workspace.sh 2>&1 | tee -a .log/create_catkin_workspace.log'

.PHONY: create_catkin_workspace
create_catkin_workspace: clean_catkin_workspace## Creates a catkin workspace @ adore/catkin_workspace. Can be called within the adore-cli or on the host.
	@if [ -f "/.dockerenv" ]; then\
            bash tools/create_catkin_workspace.sh;\
            exit 0;\
        else\
            make create_catkin_workspace_docker;\
            exit 0;\
        fi;

.PHONY: build_adore-cli_fast
build_adore-cli_fast: # build adore-cli if it does not already exist in the docker repository. If it does exist this is a noop.
	@[ -n "$$(docker images -q adore-cli:latest)" ] || \
    make build_adore-cli 

.PHONY: build_adore-cli
build_adore-cli: build_catkin_base build_plotlabserver ## Builds the ADORe CLI docker context/image
	docker compose build adore-cli \
                         --build-arg UID=${UID} \
                         --build-arg GID=${GID} \
                         --build-arg DOCKER_GID=${DOCKER_GID}
	docker compose build adore-cli-x11-display \
                         --build-arg UID=${UID} \
                         --build-arg GID=${GID} \
                         --build-arg DOCKER_GID=${DOCKER_GID}

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

.PHONY: run_test_scenarios
run_test_scenarios: adore-cli_setup adore-cli_start_headless adore-cli_scenarios_run adore-cli_teardown
