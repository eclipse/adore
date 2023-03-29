# This Makefile contains useful targets that can be included in downstream projects.

ifndef LIBADORE_MAKEFILE_PATH

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
LIBADORE_PROJECT:=libadore

LIBADORE_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
MAKE_GADGETS_PATH:=${LIBADORE_MAKEFILE_PATH}/make_gadgets
REPO_DIRECTORY:=${LIBADORE_MAKEFILE_PATH}

LIBADORE_TAG:=$(shell cd "${MAKE_GADGETS_PATH}" && make get_sanitized_branch_name REPO_DIRECTORY="${REPO_DIRECTORY}")
LIBADORE_IMAGE:=${LIBADORE_PROJECT}:${LIBADORE_TAG}

LIBADORE_CMAKE_BUILD_PATH:="${LIBADORE_PROJECT}/build"
LIBADORE_CMAKE_INSTALL_PATH:="${LIBADORE_CMAKE_BUILD_PATH}/install"


.PHONY: build_libadore 
build_libadore: ## Build libadore
	cd "${LIBADORE_MAKEFILE_PATH}" && make

.PHONY: test_libadore 
test_libadore: ## run libadore unit tests 
	cd "${LIBADORE_MAKEFILE_PATH}" && make test

.PHONY: clean_libadore
clean_libadore: ## Clean libadore build artifacts
	cd "${LIBADORE_MAKEFILE_PATH}" && make clean

.PHONY: branch_libadore
branch_libadore: ## Returns the current docker safe/sanitized branch for libadore
	@printf "%s\n" ${LIBADORE_TAG}

.PHONY: image_libadore
image_libadore: ## Returns the current docker image name for libadore
	@printf "%s\n" ${LIBADORE_IMAGE}

.PHONY: update_libadore
update_libadore:
	cd "${LIBADORE_MAKEFILE_PATH}" && git pull

endif
