#!/usr/bin/env bash

function echoerr { echo "$@" >&2; exit 1;}
SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

sudo python -m pip install docnado --upgrade
rm -rf docs || true
mkdir docs

source config.env
git pull origin $GIT_BRANCH
git checkout $GIT_BRANCH -- documentation

for docfile in documentation/*.md; do
    docfile=$(basename $docfile)
    envsubst < documentation/$docfile > docs/$docfile
done

cd docs && ln -s README.md home.md

cd "${SCRIPT_DIRECTORY}"
docnado --html docs
cd docs/w
ln -s home.html index.html

git add docs
