SHELL:=/bin/bash

.DEFAULT_GOAL := all

.PHONY: build

MAKEFLAGS += --no-print-directory

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
XODR_PROJECT="xodr"
XODR_VERSION="latest"
XODR_TAG="${XODR_PROJECT}:${XODR_VERSION}"

CSAPS_CPP_PROJECT="csaps-cpp"
CSAPS_CPP_VERSION="latest"
CSAPS_CPP_TAG="${CSAPS_CPP_PROJECT}:${CSAPS_CPP_VERSION}"

DOCKER_REPOSITORY="andrewkoerner/adore"

EXTERNALS_DOCKER_ARCHIVE="/var/tmp/libadore_externals.tar"

.PHONY: save_docker_images
save_docker_images:
	@docker save -o "${EXTERNALS_DOCKER_ARCHIVE}" "${QPOASES_TAG}" "${DLIB_TAG}" "${CATCH2_TAG}" "${OSQP_TAG}" "${XODR_TAG}"

.PHONY: load_docker_images
load_docker_images:
	@docker load --input "${EXTERNALS_DOCKER_ARCHIVE}" 2>/dev/null || true

.PHONY: build_xodr
build_xodr:
	cd xodr && \
    make
.PHONY: build_fast_xodr
build_fast_xodr:
	@if [ -n "$$(docker images -q ${XODR_TAG})" ]; then \
        echo "Docker image: ${XODR_TAG} already build, skipping build."; \
    else \
        make build_xodr;\
    fi

.PHONY: build_qpOASES
build_qpOASES: set_qpOASES_env build
set_qpOASES_env: 
	$(eval PROJECT := ${QPOASES_PROJECT})
	$(eval TAG := ${QPOASES_TAG})
.PHONY: build_fast_qpOASES
build_fast_qpOASES:
	@if [ -n "$$(docker images -q ${QPOASES_TAG})" ]; then \
        echo "Docker image: ${QPOASES_TAG} already build, skipping build."; \
    else \
        make build_xodr;\
    fi

.PHONY: build_dlib
build_dlib: set_dlib_env build
set_dlib_env: 
	$(eval PROJECT := ${DLIB_PROJECT}) 
	$(eval TAG := ${DLIB_TAG})
.PHONY: build_fast_dlib
build_fast_dlib:
	@if [ -n "$$(docker images -q ${DLIB_TAG})" ]; then \
        echo "Docker image: ${DLIB_TAG} already build, skipping build."; \
    else \
        make build_dlib;\
    fi

.PHONY: build_csaps-cpp
build_csaps-cpp:
	cd csaps-cpp && \
    make
.PHONY: build_fast_csaps-cpp
build_fast_csaps:
	@if [ -n "$$(docker images -q ${CSAPS_CPP_TAG})" ]; then \
        echo "Docker image: ${CSAPS_CPP_TAG} already build, skipping build."; \
    else \
        make build_csaps;\
    fi

.PHONY: build_Catch2
build_Catch2: set_Catch2_env build
set_Catch2_env: 
	$(eval PROJECT := ${CATCH2_PROJECT}) 
	$(eval TAG := ${CATCH2_TAG})
.PHONY: build_fast_Catch2
build_fast_Catch2:
	@if [ -n "$$(docker images -q ${CATCH2_TAG})" ]; then \
        echo "Docker image: ${CATCH2_TAG} already build, skipping build."; \
    else \
        make build_Catch2;\
    fi

.PHONY: build_osqp
build_osqp: set_osqp_env build
set_osqp_env: 
	$(eval PROJECT := ${OSQP_PROJECT}) 
	$(eval TAG := ${OSQP_TAG})
.PHONY: build_fast_osqp
build_fast_osqp:
	@if [ -n "$$(docker images -q ${OSQP_TAG})" ]; then \
        echo "Docker image: ${OSQP_TAG} already build, skipping build."; \
    else \
        make build_osqp;\
    fi

.PHONY: all
all:
	make load_docker_images
	make docker_pull_fast
	make build_fast_xodr
	make build_fast_qpOASES
	make build_fast_dlib
	make build_fast_csaps-cpp
	make build_fast_Catch2
	make build_fast_osqp
	make save_docker_images

.PHONY: build
build:
	rm -rf ${PROJECT}/build
	docker build --network host \
                 --tag $(shell echo ${TAG} | tr A-Z a-z) \
                 --build-arg PROJECT=${PROJECT} .
	docker cp $$(docker create --rm $(shell echo ${TAG} | tr A-Z a-z)):/tmp/${PROJECT}/build ${PROJECT}

.PHONY: clean
clean:
	cd "${ROOT_DIR}" && rm -rf $$(find . -name build -type d)
	docker rm $$(docker ps -a -q --filter "ancestor=${OSQP_TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${OSQP_TAG}) 2> /dev/null || true
	docker rm $$(docker ps -a -q --filter "ancestor=${QPOASES_TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${QPOASES_TAG}) 2> /dev/null || true
	docker rm $$(docker ps -a -q --filter "ancestor=${DLIB_TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${DLIB_TAG}) 2> /dev/null || true
	docker rm $$(docker ps -a -q --filter "ancestor=${CSAPS_CPP_TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${CSAPS_CPP_TAG}) 2> /dev/null || true
	docker rm $$(docker ps -a -q --filter "ancestor=${CATCH2_TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${CATCH2_TAG}) 2> /dev/null || true
	cd csaps-cpp && make clean
	cd xodr && make clean

.PHONY: docker_publish
docker_publish:
	docker tag "${QPOASES_TAG}" "${DOCKER_REPOSITORY}:${QPOASES_PROJECT}_${QPOASES_VERSION}"
	docker push "${DOCKER_REPOSITORY}:${QPOASES_PROJECT}_${QPOASES_VERSION}"

	docker tag "${DLIB_TAG}" "${DOCKER_REPOSITORY}:${DLIB_PROJECT}_${DLIB_VERSION}"
	docker push "${DOCKER_REPOSITORY}:${DLIB_PROJECT}_${DLIB_VERSION}"

	docker tag "${OSQP_TAG}" "${DOCKER_REPOSITORY}:${OSQP_PROJECT}_${OSQP_VERSION}"
	docker push "${DOCKER_REPOSITORY}:${OSQP_PROJECT}_${OSQP_VERSION}"
	
	docker tag "${CSAPS_CPP_TAG}" "${DOCKER_REPOSITORY}:${CSAPS_CPP_PROJECT}_${CSAPS_CPP_VERSION}"
	docker push "${DOCKER_REPOSITORY}:${CSAPS_CPP_PROJECT}_${CSAPS_CPP_VERSION}"

	docker tag "${XODR_TAG}" "${DOCKER_REPOSITORY}:${XODR_PROJECT}_${XODR_VERSION}"
	docker push "${DOCKER_REPOSITORY}:${XODR_PROJECT}_${XODR_VERSION}"

	docker tag "${CATCH2_TAG}" "${DOCKER_REPOSITORY}:${CATCH2_PROJECT}_${CATCH2_VERSION}"
	docker push "${DOCKER_REPOSITORY}:${CATCH2_PROJECT}_${CATCH2_VERSION}"

.PHONY: docker_pull
docker_pull:
	docker pull "${DOCKER_REPOSITORY}:${QPOASES_PROJECT}_${QPOASES_VERSION}"
	docker tag "${DOCKER_REPOSITORY}:${QPOASES_PROJECT}_${QPOASES_VERSION}" "${QPOASES_TAG}"
	docker rmi "${DOCKER_REPOSITORY}:${QPOASES_PROJECT}_${QPOASES_VERSION}"

	docker pull "${DOCKER_REPOSITORY}:${DLIB_PROJECT}_${DLIB_VERSION}"
	docker tag "${DOCKER_REPOSITORY}:${DLIB_PROJECT}_${DLIB_VERSION}" "${DLIB_TAG}"
	docker rmi "${DOCKER_REPOSITORY}:${DLIB_PROJECT}_${DLIB_VERSION}"

	docker pull "${DOCKER_REPOSITORY}:${OSQP_PROJECT}_${OSQP_VERSION}"
	docker tag "${DOCKER_REPOSITORY}:${OSQP_PROJECT}_${OSQP_VERSION}" "${OSQP_TAG}"
	docker rmi "${DOCKER_REPOSITORY}:${OSQP_PROJECT}_${OSQP_VERSION}"
	
	docker pull "${DOCKER_REPOSITORY}:${CSAPS_CPP_PROJECT}_${CSAPS_CPP_VERSION}"
	docker tag "${DOCKER_REPOSITORY}:${CSAPS_CPP_PROJECT}_${CSAPS_CPP_VERSION}" "${CSAPS_CPP_TAG}"
	docker rmi "${DOCKER_REPOSITORY}:${CSAPS_CPP_PROJECT}_${CSAPS_CPP_VERSION}"

	docker pull "${DOCKER_REPOSITORY}:${XODR_PROJECT}_${XODR_VERSION}"
	docker tag "${DOCKER_REPOSITORY}:${XODR_PROJECT}_${XODR_VERSION}" "${XODR_TAG}"
	docker rmi "${DOCKER_REPOSITORY}:${XODR_PROJECT}_${XODR_VERSION}"

	docker pull "${DOCKER_REPOSITORY}:${CATCH2_PROJECT}_${CATCH2_VERSION}"
	docker tag "${DOCKER_REPOSITORY}:${CATCH2_PROJECT}_${CATCH2_VERSION}" "${CATCH2_TAG}"
	docker rmi "${DOCKER_REPOSITORY}:${CATCH2_PROJECT}_${CATCH2_VERSION}"

.PHONY: docker_pull_fast
docker_pull_fast:
	@[ -n "$$(docker images -q ${QPOASES_TAG})" ] || make docker_pull
	@[ -n "$$(docker images -q ${DLIB_TAG})" ] || make docker_pull
	@[ -n "$$(docker images -q ${OSQP_TAG})" ] || make docker_pull
	@[ -n "$$(docker images -q ${CSAPS_CPP_TAG})" ] || make docker_pull
	@[ -n "$$(docker images -q ${XODR_TAG})" ] || make docker_pull
	@[ -n "$$(docker images -q ${CATCH2_TAG})" ] || make docker_pull

