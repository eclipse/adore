SHELL:=/bin/bash

.DEFAULT_GOAL := build_gh-pages

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

USER := $(shell whoami)
UID := $(shell id -u)
GID := $(shell id -g)

.PHONY: help  
help:
	@printf "Usage: make \033[36m<target>\033[0m\n%s\n" "$$(awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z0-9_-]+:.*?##/ { printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2 } /^##@/ { printf "\n\033[1m%s\033[0m\n", substr($$0, 5) } ' $(MAKEFILE_LIST) | sort | uniq)"

.PHONY: all
all: build_doxygen_docs build_sphinx_docs 

.PHONY: update_submodules
update_submodules:
	git submodule update --init

.PHONY: build_doxygen_docs 
build_doxygen_docs: ## Build doxygen documentation. Requires doxygen to be installed
	@command -v doxygen >/dev/null 2>&1 || { echo >&2 "Error: doxygen is not installed. Install with: 'sudo apt-get install -y doxygen'."; exit 1; }
	cd doxygen && doxygen Doxyfile-project.cfg

.PHONY: build_sphinx_docs
build_sphinx_docs: ## Build sphinx documentation 
	command -v sphinx-build >/dev/null 2>&1 || { echo >&2 "Error: sphinx-build is not installed. Install with: 'sudo apt-get install -y sphinx'."; exit 1; }
	python -c "import breathe" 2>/dev/null || { echo >&2 "Error: sphinx extension breathe is not installed. Install with: 'python3 -m pip install breathe'."; exit 1; }
	python -c "import exhale" 2>/dev/null || { echo >&2 "Error: sphinx extension exhale is not installed. Install with: 'python3 -m pip install exhale'."; exit 1; }

	cd sphinx && make html

.PHONY: build_gh-pages_docnado
build_gh-pages_docnado: build_doxygen_docs build_docnado_docs
	rm -rf docs
	mkdir -p docs
	cp -r landing_page/* docs/
	cp -r docnado/html docs/docnado
	cp -r doxygen/generated_doxygen_documentation/html docs/docnado/w/generated_doxygen_documentation

.PHONY: build_gh-pages
build_gh-pages: build_doxygen_docs build_mkdocs
	rm -rf docs
	mkdir -p docs
	cp -r landing_page/* docs/
	cp -r mkdocs/site docs/mkdocs
	cp -r doxygen/generated_doxygen_documentation/html docs/generated_doxygen_documentation

.PHONY: populate_module_docs 
populate_module_docs: update_submodules
	bash populate_module_readme_docs.sh

.PHONY: build_mkdocs
build_mkdocs: populate_module_docs ## Build mkdocs 
	rm mkdocs/docs -rf
	rm mkdocs/site -rf
	cp -r technical_reference_manual mkdocs/docs
	cd mkdocs && docker build -f Dockerfile.mkdocs -t mkdocs:latest .
	docker run --user ${UID}:${GID} -v ${ROOT_DIR}/mkdocs:/tmp/docs mkdocs:latest

.PHONY: serve_nginx
serve_nginx: build_gh-pages
	rm mkdocs/docs -rf
	rm mkdocs/site -rf
	cp docs mkdocs -r
	docker rm --force adore-docs
	cd docs && docker run -it --rm --name "adore-docs" -d -p 80:80 -v ${ROOT_DIR}/docs:/usr/share/nginx/html nginx:latest
	sleep 1s
	firefox http://localhost
	@echo "Docs can be found at: http://localhost" 

.PHONY: publish_gh-pages
publish_gh-pages: build_gh-pages
	bash publish_gh-pages.sh


.PHONY: publish
publish: publish_gh-pages ## Publish docs to gh-pages branch

.PHONY: build
build: build_gh-pages ## Build docs 

.PHONY: serve
serve: serve_nginx ## Build and serve docs to http://localhost
