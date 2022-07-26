include make_gadgets/Makefile

SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=


CMAKE_PREFIX_PATH?=$(shell realpath $$(find . -name install | grep "install" | grep -e "install/\|CPack" -v) | tr '\n' ';') 

.PHONY: set_mode 
set_mode:
ifeq ($(DEBUG), 1)
	$(eval DOCKER_BUILDKIT:=0)
else
	$(eval DOCKER_BUILDKIT:=1)
endif

PROJECT="adore_if_ros"
VERSION="latest"
TAG="${PROJECT}:${VERSION}"

.PHONY: all 
all: root_check docker_group_check build

.PHONY: build 
build: set_mode ## Build adore_if_ros
	rm -rf ${PROJECT}/build
	cd adore_if_ros_msg && \
	make
	cd plotlablib && \
	make
	docker build --tag $(shell echo ${TAG} | tr A-Z a-z) \
                 --network host \
                 --build-arg DEBUG="${DEBUG}" \
                 --build-arg PROJECT=${PROJECT} .
	mkdir -p "${ROOT_DIR}/tmp/${PROJECT}/build"
	docker cp $$(docker create --rm $(shell echo ${TAG} | tr A-Z a-z)):/tmp/${PROJECT}/${PROJECT}/build tmp/${PROJECT}/build
	cp -r "${ROOT_DIR}/tmp/${PROJECT}/build/build" "${ROOT_DIR}/${PROJECT}"
	rm -rf ${ROOT_DIR}/tmp

.PHONY: clean 
clean:
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	docker rm $$(docker ps -a -q --filter "ancestor=${TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${TAG}) 2> /dev/null || true
	docker rmi ${TAG} --force 2> /dev/null

.PHONY: clean_docker 
clean_docker:
	docker rmi $$(docker images --filter "dangling=true" -q) --force

.PHONY: lint 
lint:
	cd cpplint && \
        make lint CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/adore_if_ros)

.PHONY: lintfix 
lintfix:
	cd cpplint && \
        make lintfix CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/adore_if_ros)

.PHONY: lintfix_simulate 
lintfix_simulate:
	cd cpplint && \
        make lintfix_simulate CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/adore_if_ros)

.PHONY: cppcheck 
cppcheck:
	cd cppcheck && \
        make cppcheck CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/adore_if_ros); echo "error code $${?}"

.PHONY: lizard 
lizard:
	cd lizard && \
        make lizard CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/adore_if_ros); echo "error code $${?}"

.PHONY: build_docker_layers 
build_docker_layers:
	@DOCKER_BUILDKIT=0 make build #| grep "\-\-\->" | \
                                       grep -v "Using" | \
                                       sed "s| \-\-\-> ||g" | \
                                       sed "s|Running in ||g" | \
                                       tr '\n' ' ' >> .docker_layers_cache
	@echo "($$(git rev-parse --abbrev-ref HEAD):$$(git rev-parse --short HEAD))" >> .docker_layers_cache

