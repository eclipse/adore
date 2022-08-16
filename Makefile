SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))


.PHONY: all
all: generate_docs generate_pelican

.PHONY: generate_docs
generate_docs:
	bash generate_docs.sh

.PHONY: generate_pelican
generate_pelican:
	cd pelican && \
    make html && \
    grep -rl "/pages/Documentation.html" | xargs sed -i "s|/pages/Documentation.html|/docs/index.html|g" && \
    cp ../docs output -r

.PHONY: gh-pages_publish
gh-pages_publish:
	rm -r docs
	cp -r pelican/output docs
	git add docs/*
	git commit -am "updated docs"
	git push


.PHONY: build
build: all

.PHONY: clean
clean: 
	echo todo

