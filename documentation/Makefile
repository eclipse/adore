SHELL:=/bin/bash

.DEFAULT_GOAL := build_gh-pages

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")

MAKEFLAGS += --no-print-directory

.PHONY: help  
help:
	@printf "Usage: make \033[36m<target>\033[0m\n%s\n" "$$(awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z0-9_-]+:.*?##/ { printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2 } /^##@/ { printf "\n\033[1m%s\033[0m\n", substr($$0, 5) } ' $(MAKEFILE_LIST) | sort | uniq)"

.PHONY: all
all: build_doxygen_docs build_sphinx_docs 

.PHONY: build_doxygen_docs 
build_doxygen_docs: ## Build doxygen documentation. Requires doxygen to be installed
	command -v doxygen >/dev/null 2>&1 || { echo >&2 "Error: doxygen is not installed. Install with: 'sudo apt-get install -y doxygen'."; exit 1; }
	cd doxygen && doxygen Doxyfile-project.cfg

.PHONY: build_sphinx_docs
build_sphinx_docs: ## Build sphinx documentation 
	command -v sphinx-build >/dev/null 2>&1 || { echo >&2 "Error: sphinx-build is not installed. Install with: 'sudo apt-get install -y sphinx'."; exit 1; }
	python -c "import breathe" 2>/dev/null || { echo >&2 "Error: sphinx extension breathe is not installed. Install with: 'python3 -m pip install breathe'."; exit 1; }
	python -c "import exhale" 2>/dev/null || { echo >&2 "Error: sphinx extension exhale is not installed. Install with: 'python3 -m pip install exhale'."; exit 1; }

	cd sphinx && make html

.PHONY: build_docnado_docs
build_docnado_docs:  ## Build docnado documentation aka gh-pages webpage 
	cd docnado && bash generate_docnado_docs.sh

.PHONY: build_gh-pages
build_gh-pages: build_doxygen_docs build_docnado_docs
	rm -rf docs
	mkdir -p docs
	cp -r landing_page/* docs/
	cp -r docnado/html docs/docnado
	cp -r doxygen/generated_doxygen_documentation/html docs/docnado/w/generated_doxygen_documentation

.PHONY: publish_gh-pages
publish_gh-pages: build_gh-pages ## Publish docs to gh-pages branch
	bash publish_gh-pages.sh

.PHONY: publish
publish: publish_gh-pages ## Publish docs to gh-pages branch

.PHONY: build
build: build_gh-pages ## Publish docs to gh-pages branch
