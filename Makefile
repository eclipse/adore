SHELL:=/bin/bash

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")

include make_gadgets/make_gadgets.mk

DOCKER_COMPOSE_FILE?=${ROOT_DIR}/docker-compose.yaml

.EXPORT_ALL_VARIABLES:
CATKIN_WORKSPACE_DIRECTORY:=${ROOT_DIR}/catkin_workspace
SOURCE_DIRECTORY:=${ROOT_DIR}
ADORE_CLI_WORKING_DIRECTORY:=${ROOT_DIR}
SUBMODULES_PATH:=${ROOT_DIR}

ifeq ($(APT_CACHER_NG_ENABLED),)
DOCKER_CONFIG?=${ROOT_DIR}/apt_cacher_ng_docker
endif


include adore_cli/adore_cli.mk
include ${SUBMODULES_PATH}/ci_teststand/ci_teststand.mk

PROJECT:="adore"
TAG:=$(shell cd make_gadgets && make get_sanitized_branch_name REPO_DIRECTORY=${ROOT_DIR})
.PHONY: set_env 
set_env:
	$(eval PROJECT := "${PROJECT}") 
	$(eval TAG := "${TAG}")

.PHONY: clean
clean:
	bash adore_tools/tools/clean_all.sh
	rm -rf "${ROOT_DIR}/${PROJECT}"

.PHONY: build
build: build_core

.PHONY: build_core
build_core: start_apt_cacher_ng build_adore_if_ros build_fast_adore-cli ## Build core ADORe modules. \\e[1;4;38;5;208;48;5;15mWarning:\\e[0m Any changes to the adore cli context require manual re-build with 'make build_adore-cli'
	mkdir -p "${ROOT_DIR}/${PROJECT}/build"

.PHONY: build_all
build_all: start_apt_cacher_ng ## Build all ADORe modules. \\e[1;4;38;5;208;48;5;15mWarning:\\e[0m Any changes to the adore cli context require manual re-build with 'make build_adore-cli'
	mkdir -p "${ROOT_DIR}/${PROJECT}/build"
	bash adore_tools/tools/build_all.sh

.PHONY: lint_all 
lint_all: ## Run linting for all modules
	$(warning )
	mkdir -p .log
	find . -name "**lint_report.log" -exec rm -rf {} \;
	EXIT_STATUS=0; \
        (cd sumo_if_ros && make lint) || EXIT_STATUS=$$? && \
        (cd libadore && make lint) || EXIT_STATUS=$$? && \
        (cd adore_if_ros && make lint) || EXIT_STATUS=$$? && \
        find . -name "**_lint_report.log" -exec mv {} .log/ \; && \
        exit $$EXIT_STATUS
 
.PHONY: cppcheck_all 
cppcheck_all: ## Run cppcheck for all modules
	mkdir -p .log
	find . -name "**cppcheck_report.log" -exec rm -rf {} \;
	EXIT_STATUS=0; \
        (cd sumo_if_ros && make cppcheck) || EXIT_STATUS=$$? && \
        (cd libadore && make cppcheck) || EXIT_STATUS=$$? && \
        (cd adore_if_ros && make cppcheck) || EXIT_STATUS=$$? && \
        find . -name "**_cppcheck_report.log" -exec mv {} .log/ \; && \
        exit $$EXIT_STATUS

.PHONY: lizard_all 
lizard_all: ## Run lizard for all modules
	mkdir -p .log
	find . -name "**lizard_report.log" -exec rm -rf {} \;
	find . -name "**lizard_report.xml" -exec rm -rf {} \;
	EXIT_STATUS=0; \
        (cd sumo_if_ros && make lizard) || EXIT_STATUS=$$? && \
        (cd libadore && make lizard) || EXIT_STATUS=$$? && \
        (cd adore_if_ros && make lizard) || EXIT_STATUS=$$? && \
        find . -name "**_lizard_report.log" -exec mv {} .log/ \; && \
        find . -name "**_lizard_report.xml" -exec mv {} .log/ \; && \
        exit $$EXIT_STATUS

.PHONY: test_all 
test_all: ## Run unit tests for all supportingll modules
	mkdir -p .log
	find . -name "_ctest.log" -exec rm -rf {} \;
	EXIT_STATUS=0; \
        (cd libadore && make test) || EXIT_STATUS=$$? && \
        find . -name "**_ctest.log" -exec mv {} .log/ \; && \
        exit $$EXIT_STATUS

.PHONY: test
test: test_all

