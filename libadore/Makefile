SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

.PHONY: help  
help:
	@awk 'BEGIN {FS = ":.*##"; printf "Usage: make \033[36m<target>\033[0m\n"} /^[a-zA-Z_-]+:.*?##/ { printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2 } /^##@/ { printf "\n\033[1m%s\033[0m\n", substr($$0, 5) } ' $(MAKEFILE_LIST)

LIBADORE_PROJECT="libadore"
LIBADORE_VERSION="latest"
LIBADORE_TAG="${LIBADORE_PROJECT}:${LIBADORE_VERSION}"

.PHONY: build_libadore  
build_libadore: set_libadore_env build

.PHONY: set_libadore_env 
set_libadore_env: 
	$(eval PROJECT := ${LIBADORE_PROJECT}) 
	$(eval TAG := ${LIBADORE_TAG})

.PHONY: all 
all: build_external build_libadore

.PHONY: build 
build: all

.PHONY: clean_libadore 
clean_libadore:
	rm -rf "${ROOT_DIR}/libadore/build"

.PHONY: build 
build:
	rm -rf ${PROJECT}/build
	docker compose build libadore_build
	docker compose build libadore
	docker compose build libadore_test
	mkdir -p "${ROOT_DIR}/tmp/${PROJECT}/build"
	docker cp $$(docker create --rm $(shell echo ${TAG} | tr A-Z a-z)):/tmp/${PROJECT}/build tmp/${PROJECT}/build/
	cp -r "${ROOT_DIR}/tmp/${PROJECT}/build/build" "${ROOT_DIR}/${PROJECT}"
	rm -rf ${ROOT_DIR}/tmp

.PHONY: test 
test:
	@if [[ `docker images -q ${LIBADORE_TAG} 2> /dev/null` == "" ]]; then \
            echo "ERROR: Docker image: ${LIBADORE_TAG} does not exist. Run make build on libadore and try again." 1>&2; \
            exit 1; \
        fi;

	docker run --rm libadore_test:latest /bin/bash -c 'cd build && ctest' 2>&1 | tee -a libadore/build/ctest.log
	@echo "ctest log can be found at libadore/build/ctest.log"

	@if grep -q "Errors while running CTest" "libadore/build/ctest.log"; then \
            exit 1; \
        fi;

.PHONY: build_external 
make build_external:
	cd "${ROOT_DIR}/libadore/external/" && make

.PHONY: clean_external 
make clean_external:
	cd "${ROOT_DIR}/libadore/external/" && make clean

.PHONY: clean 
clean: clean_external clean_libadore
	docker rm $$(docker ps -a -q --filter "ancestor=${LIBADORE_TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${LIBADORE_TAG}) 2> /dev/null || true

.PHONY: clean_docker 
clean_docker:
	docker rmi $$(docker images --filter "dangling=true" -q) --force
	docker rmi ${LIBADORE_TAG} --force

.PHONY: lint 
lint:
	cd cpplint && \
        make lint CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/libadore/adore)

.PHONY: lintfix 
lintfix:
	cd cpplint && \
        make lintfix CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/libadore/adore)

.PHONY: lintfix_simulate 
lintfix_simulate:
	cd cpplint && \
        make lintfix_simulate CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/libadore/adore)

.PHONY: cppcheck
cppcheck:
	cd cppcheck && \
        make cppcheck CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/libadore/adore)

.PHONY: lizard
lizard:
	cd lizard && \
        make lizard CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/libadore/adore)
