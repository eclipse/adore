SHELL:=/bin/bash

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")

include make_gadgets/make_gadgets.mk

.EXPORT_ALL_VARIABLES:
CATKIN_WORKSPACE_DIRECTORY:=${ROOT_DIR}/catkin_workspace
SOURCE_DIRECTORY:=${ROOT_DIR}
ADORE_CLI_WORKING_DIRECTORY:=${ROOT_DIR}
SUBMODULES_PATH:=${ROOT_DIR}

include adore_cli/adore_cli.mk

.PHONY: lint 
lint: ## Run linting for all modules
	mkdir -p .log
	find . -name "**lint_report.log" -exec rm -rf {} \;
	EXIT_STATUS=0; \
        (cd sumo_if_ros && make lint) || EXIT_STATUS=$$? && \
        (cd libadore && make lint) || EXIT_STATUS=$$? && \
        (cd adore_if_ros && make lint) || EXIT_STATUS=$$? && \
        find . -name "**_lint_report.log" -exec mv {} .log/ \; && \
        exit $$EXIT_STATUS
 
.PHONY: cppcheck 
cppcheck: ## Run cppcheck for all modules
	mkdir -p .log
	find . -name "**cppcheck_report.log" -exec rm -rf {} \;
	EXIT_STATUS=0; \
        (cd sumo_if_ros && make cppcheck) || EXIT_STATUS=$$? && \
        (cd libadore && make cppcheck) || EXIT_STATUS=$$? && \
        (cd adore_if_ros && make cppcheck) || EXIT_STATUS=$$? && \
        find . -name "**_cppcheck_report.log" -exec mv {} .log/ \; && \
        exit $$EXIT_STATUS

.PHONY: lizard 
lizard: ## Run lizard for all modules
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
