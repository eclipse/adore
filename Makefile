SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")

include adore_if_ros_msg/make_gadgets/make_gadgets.mk
include adore_if_ros_msg/make_gadgets/docker/docker-tools.mk
include adore_if_ros_msg/make_gadgets/docker/docker-image-cacher.mk
include adore-cli.mk
#include adore_if_ros/adore_if_ros.mk
#include adore_if_ros_msg/adore_if_ros_msg.mk
include v2x_if_ros_msg/v2x_if_ros_msg.mk
#include plotlabserver/plotlabserver.mk
#include catkin_base.mk


.EXPORT_ALL_VARIABLES:
DOCKER_IMAGE_EXCLUSION_LIST="adore_if_ros:latest adore_if_ros_msg:latest plotlablib:latest plotlabserver:latest plotlabserver_build:latest v2x_if_ros_msg:latest libzmq:latest csaps-cpp:latest adore-cli:latest carlasim/carla:0.9.13"
DOCKER_IMAGE_INCLUSION_LIST="edrevo/dockerfile-plus:latest ubuntu:20.04 ubuntu:focal"
DOCKER_IMAGE_CACHE_DIRECTORY="${ROOT_DIR}/.docker_image_cache"
DOCKER_IMAGE_SEARCH_PATH=${ROOT_DIR}

include apt_cacher_ng_docker/apt_cacher_ng_docker.mk

.PHONY: all
all: help 

.PHONY: build 
build: _build ## Build ADORe 

.PHONY: build_adore_if_ros_msg
build_adore_if_ros_msg:
	cd adore_if_ros_msg && \
	make build

.PHONY: build_adore_v2x
build_adore_if_v2x:
	cd adore_if_v2x && \
    APT_CACHER_NG_DOCKER_MAKEFILE_PATH= make build


.PHONY: _build 
_build: \
        docker_load \
        docker_host_context_check \
        docker_storage_inventory_prebuild \
        start_apt_cacher_ng \
        build_adore_if_ros_msg \
        build_v2x_if_ros_msg \
        build_adore_if_ros \
        build_adore_if_v2x \
        docker_storage_inventory_postbuild \
        clean_up 

.PHONY: clean 
clean: docker_delete_all_build_cache
	cd adore_if_ros && make clean
	cd libadore && make clean
	cd sumo_if_ros && make clean
	cd adore_if_ros_msg && make clean
	cd v2x_if_ros_msg && make clean
	cd adore_if_v2x && make clean
	cd plotlabserver && make clean


.PHONY: clean_up
clean_up: stop_apt_cacher_ng docker_save_images

.PHONY: docker_storage_inventory_prebuild
docker_storage_inventory_prebuild:
	mkdir -p .log
	bash tools/docker_storage_inventory.sh --log-directory .log 

.PHONY: docker_storage_inventory_postbuild
docker_storage_inventory_postbuild:
	bash tools/docker_storage_inventory.sh --log-directory .log

.PHONY: docker_save_images
docker_save_images:
	@nohup sh -c 'make docker_save' > /dev/null 2>&1 &

.PHONY: docker_load_images
docker_load_images:
	@nohup make docker_load > /dev/null 2>&1 &

.PHONY: test 
test: ## Run ADORe unit tests
	mkdir -p .log && \
    cd libadore && \
	make test | tee "${ROOT_DIR}/.log/libadore_unit_test.log"; exit $$PIPESTATUS

.PHONY: lint 
lint: ## Run linting for all modules
	mkdir -p .log
	find . -name "**lint_report.log" -exec rm -rf {} \;
	EXIT_STATUS=0; \
        (cd sumo_if_ros && make lint) || EXIT_STATUS=$$? && \
        (cd libadore && make lint) || EXIT_STATUS=$$? && \
        (cd adore_if_ros && make lint) || EXIT_STATUS=$$? && \
        find sumo_if_ros -type f -name 'lint_report.log' -exec mv {} .log/sumo_if_ros_lint_report.log \; && \
        find libadore -type f -name 'lint_report.log' -exec mv {} .log/libadore_lint_report.log \; && \
        find adore_if_ros -type f -name 'lint_report.log' -exec mv {} .log/adore_if_ros_lint_report.log \; && \
        exit $$EXIT_STATUS
 
.PHONY: lizard 
lizard: ## Run lizard static analysis tool for all modules
	mkdir -p .log
	find . -name "**lizard_report.**" -exec rm -rf {} \;
	unset -f DOCKER_CONFIG && \
    EXIT_STATUS=0; \
        (cd sumo_if_ros && make lizard) || EXIT_STATUS=$$? && \
        (cd libadore && make lizard) || EXIT_STATUS=$$? \ && \
        (cd adore_if_ros && make lizard) || EXIT_STATUS=$$? && \
        find sumo_if_ros -type f -name 'lizard_report.log' -exec mv {} .log/sumo_if_ros_lizard_report.log \; && \
        find sumo_if_ros -type f -name 'lizard_report.xml' -exec mv {} .log/sumo_if_ros_lizard_report.xml \; && \
        find libadore -type f -name 'lizard_report.log' -exec mv {} .log/libadore_lizard_report.log \; && \
        find libadore -type f -name 'lizard_report.xml' -exec mv {} .log/libadore_lizard_report.xml \; && \
        find adore_if_ros -type f -name 'lizard_report.log' -exec mv {} .log/adore_if_ros_lizard_report.log \; && \
        find adore_if_ros -type f -name 'lizard_report.xml' -exec mv {} .log/adore_if_ros_if_ros_lizard_report.xml \; && \
        exit $$EXIT_STATUS

.PHONY: cppcheck 
cppcheck: ## Run cppcheck static checking tool for all modules.
	mkdir -p .log
	find . -name "**cppcheck_report.log" -exec rm -rf {} \;
	EXIT_STATUS=0; \
        (cd sumo_if_ros && make cppcheck) || EXIT_STATUS=$$? && \
        (cd libadore && make cppcheck) || EXIT_STATUS=$$? && \
        (cd adore_if_ros && make cppcheck) || EXIT_STATUS=$$? && \
        find sumo_if_ros -type f -name 'cppcheck_report.log' -exec mv {} .log/sumo_if_ros_cppcheck_report.log \; && \
        find libadore -type f -name 'cppcheck_report.log' -exec mv {} .log/libadore_cppcheck_report.log \; && \
        find adore_if_ros -type f -name 'cppcheck_report.log' -exec mv {} .log/adore_if_ros_cppcheck_report.log \; && \
        exit $$EXIT_STATUS


