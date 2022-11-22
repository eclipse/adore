SHELL:=/bin/bash

.DEFAULT_GOAL := all

include adore_if_ros_msg/make_gadgets/make_gadgets.mk
include adore_if_ros_msg/make_gadgets/docker/docker-tools.mk
include adore-cli.mk
include adore_if_ros/adore_if_ros.mk
include adore_if_ros_msg/adore_if_ros_msg.mk
include catkin_base.mk
include apt_cacher_ng_docker/apt_cacher_ng_docker.mk


.EXPORT_ALL_VARIABLES:
#DOCKER_IMAGE_EXCLUSION_LIST="adore_if_ros:latest adore_if_ros_msg:latest plotlablib:latest plotlabserver:latest plotlabserver_build:latest v2x_if_ros_msg:latest libzmq:latest csaps-cpp:latest adore-cli:latest carlasim/carla:0.9.12"
#DOCKER_IMAGE_INCLUSION_LIST="edrevo/dockerfile-plus:latest"

#include plotlabserver/plotlabserver.mk

.PHONY: all
all: help 

.PHONY: build 
build: docker_host_context_check start_apt_cacher_ng build_adore_if_ros_msg build_adore_if_ros clean_up ## Build ADORe 

.PHONY: clean 
clean: clean_adore_if_ros ## Clean ADORe

.PHONY: clean_up
clean_up: stop_apt_cacher_ng

.PHONY: test 
test:
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


