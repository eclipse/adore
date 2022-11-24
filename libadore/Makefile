SHELL:=/bin/bash

.DEFAULT_GOAL := all

.PHONY: all 
all: root_check docker_group_check build

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")
MAKEFILE_PATH:=$(shell dirname "$(abspath "$(lastword $(MAKEFILE_LIST)"))")

MAKEFLAGS += --no-print-directory

include make_gadgets/make_gadgets.mk
include make_gadgets/docker/docker-tools.mk
include cppcheck_docker/cppcheck_docker.mk
include cpplint_docker/cpplint_docker.mk
include lizard_docker/lizard_docker.mk

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?= 

CPP_PROJECT_DIRECTORY:="${ROOT_DIR}/libadore/adore"

CMAKE_PREFIX_PATH?=$(shell realpath "$$(find . -type d | grep "build" | grep -v "build/")" 2>/dev/null | tr '\n' ';')

include libadore.mk

.PHONY: all
all: root_check docker_group_check build_external build

.PHONY: set_env 
set_env: 
	$(eval PROJECT := ${LIBADORE_PROJECT}) 
	$(eval TAG := ${LIBADORE_TAG})

.PHONY: build 
build: clean set_env ## Build libadore
	rm -rf "${PROJECT}/build"
	make build_external
	#docker compose build libadore_build

    #buildx bake required for builds behind proxy
	docker buildx bake libadore_build
	docker compose build libadore
	docker compose build libadore_test
	docker cp $$(docker create --rm ${PROJECT}:${TAG}):/tmp/${PROJECT}/${PROJECT}/build "${ROOT_DIR}/${PROJECT}"

.PHONY: clean 
clean: clean_external set_env ## Clean libadore
	rm -rf "${ROOT_DIR}/libadore/build"
	find . -name "**lint_report.log" -exec rm -rf {} \;
	find . -name "**cppcheck_report.log" -exec rm -rf {} \;
	find . -name "**lizard_report.xml" -exec rm -rf {} \;
	find . -name "**lizard_report.log" -exec rm -rf {} \;
	docker rm $$(docker ps -a -q --filter "ancestor=${PROJECT}:${TAG}") --force 2> /dev/null || true
	docker rmi $$(docker images -q ${PROJECT}:${TAG}) --force 2> /dev/null || true
	docker rmi $$(docker images --filter "dangling=true" -q) --force > /dev/null 2>&1 || true

.PHONY: test 
test: set_env ## run libadore unit tests
	@if [[ `docker images -q ${PROJECT}:${TAG} 2> /dev/null` == "" ]]; then \
		echo "ERROR: Docker image: ${PROJECT}:${TAG} does not exist. Run make build on libadore and try again." 1>&2; \
            exit 1; \
        fi;
	docker run --rm ${PROJECT}_test:${TAG} /bin/bash -c 'cd build && ctest' 2>&1 | tee -a libadore/build/ctest.log
	@echo "ctest log can be found at libadore/build/ctest.log"

	@if grep -q "Errors while running CTest" "libadore/build/ctest.log"; then \
            exit 1; \
     fi;

.PHONY: build_external 
build_external: ## builds all libadore external libraries 
	cd "${ROOT_DIR}/libadore/external/" && make

.PHONY: clean_external 
clean_external: ## clean all libadore external libraries
	cd "${ROOT_DIR}/libadore/external/" && make clean


.PHONY: static_checks
static_checks: lint lizard cppcheck
