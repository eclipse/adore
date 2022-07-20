SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
MAKEFLAGS += --no-print-directory

CATKIN_WORKSPACE_DIRECTORY=catkin_workspace

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=$(shell realpath ${ROOT_DIR})/apt_cacher_ng_docker

CMAKE_PREFIX_PATH?=$(shell realpath $$(find . -type d | grep "build/install" | grep -v "build/install/") | tr '\n' ';') 


DOCKER_GID := $(shell getent group | grep docker | cut -d":" -f3)
USER := $(shell whoami)
UID := $(shell id -u)
GID := $(shell id -g)

.PHONY: help
help:
	@awk 'BEGIN {FS = ":.*##"; printf "Usage: make \033[36m<target>\033[0m\n"} /^[a-zA-Z_-]+:.*?##/ { printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2 } /^##@/ { printf "\n\033[1m%s\033[0m\n", substr($$0, 5) } ' $(MAKEFILE_LIST)


.PHONY: all
all: start_apt_cacher_ng submodules_update build_sumo_if_ros build_adore_if_v2x build_adore_v2x_sim build_plotlabserver build_libadore build_adore_if_ros get_apt_cacher_ng_cache_statistics


.PHONY: build
build: all

.PHONY: clean
clean: 
	cd plotlabserver && make clean
	cd sumo_if_ros && make clean
	cd adore_if_ros_msg && make clean
	cd libadore && make clean
	cd adore_if_ros && make clean

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

.PHONY: build_plotlabserver 
build_plotlabserver: ## Build plotlabserver
	cd plotlabserver && \
    make

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
	echo ${CMAKE_PREFIX_PATH} && exit 1
	cd libadore && \
    make

.PHONY: build_sumo_if_ros 
build_sumo_if_ros: ## Build sumo_if_ros
	cd sumo_if_ros && \
    make

.PHONY: test 
test:
	mkdir -p .log && cd libadore && make test | tee ${ROOT_DIR}/.log/libadore_unit_test.log; exit $$PIPESTATUS

.PHONY: lint_sumo_if_ros 
lint_sumo_if_ros:
	cd sumo_if_ros && make lint

.PHONY: lint 
lint: ## Run linting for all modules
	EXIT_STATUS=0; \
        (cd sumo_if_ros && make lint) || EXIT_STATUS=$$? && \
        (cd libadore && make lint) || EXIT_STATUS=$$? && \
        (cd adore_if_ros && make lint) || EXIT_STATUS=$$? && \
        exit $$EXIT_STATUS
 
.PHONY: lizard 
lizard: ## Run lizard static analysis tool for all modules
	EXIT_STATUS=0; \
        (cd sumo_if_ros && make lizard) || EXIT_STATUS=$$? && \
        (cd libadore && make lizard) || EXIT_STATUS=$$? && \
        (cd adore_if_ros && make lizard) || EXIT_STATUS=$$? && \
        exit $$EXIT_STATUS

.PHONY: cppcheck 
cppcheck: ## Run cppcheck static checking tool for all modules.
	EXIT_STATUS=0; \
        (cd sumo_if_ros && make cppcheck) || EXIT_STATUS=$$? && \
        (cd libadore && make cppcheck) || EXIT_STATUS=$$? && \
        (cd adore_if_ros && make cppcheck) || EXIT_STATUS=$$? && \
        exit $$EXIT_STATUS

.PHONY: clean_catkin_workspace 
clean_catkin_workspace:
	rm -rf ${CATKIN_WORKSPACE_DIRECTORY}

.PHONY: build_catkin_base 
build_catkin_base: ## Build a docker image with base catkin tools installed with tag catkin_base:latest
	docker build --network host \
	             --file docker/Dockerfile.catkin_base \
                 --tag catkin_base \
                 --build-arg PROJECT=catkin_base .

.PHONY: create_catkin_workspace_docker
create_catkin_workspace_docker: build_catkin_base
	docker run -it \
                   --user "${UID}:${GID}" \
                   --mount type=bind,source=${ROOT_DIR},target=${ROOT_DIR} \
                   catkin_base \
                   /bin/bash -c 'cd ${ROOT_DIR} && HOME=${ROOT_DIR} CATKIN_WORKSPACE_DIRECTORY=${CATKIN_WORKSPACE_DIRECTORY} bash tools/create_catkin_workspace.sh'

.PHONY: create_catkin_workspace
create_catkin_workspace: clean_catkin_workspace## Creates a catkin workspace @ adore/catkin_workspace. Can be called within the adore-cli or on the host.
	echo "USER: ${USER}"
	@if [ "${USER}" == "adore-cli" ]; then\
            bash tools/create_catkin_workspace.sh;\
            exit 0;\
        else\
            make create_catkin_workspace_docker;\
            exit 0;\
        fi;

.PHONY: build_adore-cli
build_adore-cli: build_catkin_base ## Builds the ADORe CLI docker context/image
	COMPOSE_DOCKER_CLI_BUILD=1 docker compose build \
                                                     --build-arg UID=${UID} \
                                                     --build-arg GID=${GID} \
                                                     --build-arg DOCKER_GID=${DOCKER_GID}

.PHONY: run_ci_scenarios
run_ci_scenarios:
	bash tools/run_ci_scenarios.txt 

.PHONY: adore-cli
adore-cli:
	mkdir -p .ros/bag_files
	touch .zsh_history
	touch .zsh_history.new
	[ -n "$$(docker images -q adore-cli:latest)" ] || make build_adore-cli 
	cd plotlabserver && make stop_plotlabserver
	docker compose rm -f
	@xhost + && docker compose up --force-recreate -V -d; xhost - 
#	(cd plotlab && make start_plotlab_server_detached > /dev/null 2>&1 &);
	@docker exec -it --user adore-cli adore-cli /bin/zsh -c "bash tools/adore-cli.sh" || true
	@docker compose down && xhost - 1> /dev/null
