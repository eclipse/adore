SHELL:=/bin/bash

.DEFAULT_GOAL := all

include adore_if_ros_msg/make_gadgets/make_gadgets.mk
include adore_if_ros_msg/make_gadgets/docker/docker-tools.mk
include adore-cli.mk
include adore_if_ros/adore_if_ros.mk
include catkin_base.mk
include apt_cacher_ng_docker/apt_cacher_ng_docker.mk


.EXPORT_ALL_VARIABLES:
#DOCKER_IMAGE_EXCLUSION_LIST="adore_if_ros:latest adore_if_ros_msg:latest plotlablib:latest plotlabserver:latest plotlabserver_build:latest v2x_if_ros_msg:latest libzmq:latest csaps-cpp:latest adore-cli:latest carlasim/carla:0.9.12"
#DOCKER_IMAGE_INCLUSION_LIST="edrevo/dockerfile-plus:latest"

#include plotlabserver/plotlabserver.mk

.PHONY: all
all: help 

.PHONY: build 
build: docker_host_context_check start_apt_cacher_ng build_adore_if_ros_msg build_adore_if_ros halt ## Build ADORe 

.PHONY: clean 
clean: clean_adore_if_ros ## Clean ADORe

.PHONY: halt
clean_up: stop_apt_cacher_ng
