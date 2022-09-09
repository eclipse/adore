SHELL:=/bin/bash

.DEFAULT_GOAL := all

.PHONY: build

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

QPOASES_PROJECT=qpOASES
QPOASES_VERSION=latest
QPOASES_TAG="$(shell echo $(QPOASES_PROJECT) | tr A-Z a-z):${QPOASES_VERSION}"
DLIB_PROJECT=dlib
DLIB_VERSION=v19.19
DLIB_TAG="${DLIB_PROJECT}:${DLIB_VERSION}"
CATCH2_PROJECT="Catch2"
CATCH2_VERSION="v2.10.2"
CATCH2_TAG="catch2:${CATCH2_VERSION}"
OSQP_PROJECT="osqp"
OSQP_VERSION="latest"
OSQP_TAG="${OSQP_PROJECT}:${OSQP_VERSION}"

build_xodr:
	cd xodr && \
    make

build_qpOASES: set_qpOASES_env
#	cp files/cmake qpOASES -r
#	rm qpOASES/CMakeLists.txt
#	cp files/CMakeLists.qpOASES.txt qpOASES/CMakeLists.txt
#	touch qpOASES/README.md
#	touch qpOASES/LICENSE
	make build
set_qpOASES_env: 
	$(eval PROJECT := ${QPOASES_PROJECT})
	$(eval TAG := ${QPOASES_TAG})

build_dlib: set_dlib_env build
set_dlib_env: 
	$(eval PROJECT := ${DLIB_PROJECT}) 
	$(eval TAG := ${DLIB_TAG})

build_csaps-cpp:
	cd csaps-cpp && \
    make

build_Catch2: set_Catch2_env build
set_Catch2_env: 
	$(eval PROJECT := ${CATCH2_PROJECT}) 
	$(eval TAG := ${CATCH2_TAG})

build_osqp: set_osqp_env build
set_osqp_env: 
	$(eval PROJECT := ${OSQP_PROJECT}) 
	$(eval TAG := ${OSQP_TAG})

all: 
	make build_xodr
	make build_qpOASES
	make build_dlib
	cd csaps-cpp && make
	make build_Catch2
	make build_osqp

build:
	rm -rf ${PROJECT}/build
	docker build --network host \
                 --tag $(shell echo ${TAG} | tr A-Z a-z) \
                 --build-arg PROJECT=${PROJECT} .
	docker cp $$(docker create --rm $(shell echo ${TAG} | tr A-Z a-z)):/tmp/${PROJECT}/build ${PROJECT}

clean:
	cd "${ROOT_DIR}" && rm -rf $$(find . -name build -type d)
	docker rm $$(docker ps -a -q --filter "ancestor=${OSQP_TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${OSQP_TAG}) 2> /dev/null || true
	docker rm $$(docker ps -a -q --filter "ancestor=${QPOASES_TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${QPOASES_TAG}) 2> /dev/null || true
	docker rm $$(docker ps -a -q --filter "ancestor=${DLIB_TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${DLIB_TAG}) 2> /dev/null || true
	docker rm $$(docker ps -a -q --filter "ancestor=${CSAPS_CPP_TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${CSAPS_CPP_PROJECT}) 2> /dev/null || true
	docker rm $$(docker ps -a -q --filter "ancestor=${CATCH2_TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${CATCH2_TAG}) 2> /dev/null || true
	cd csaps-cpp && make clean
	cd xodr && make clean
