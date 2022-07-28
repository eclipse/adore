#!/usr/bin/env bash

function echoerr { echo "$@" >&2; exit 1;}
SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

pip install docnado --upgrade

rm -rf documentation || true
rm -rf docs || true
mkdir docs

main_branch=$(git symbolic-ref refs/remotes/origin/HEAD | sed 's@^refs/remotes/origin/@@')
git checkout $main_branch -- documentation
git checkout $main_branch -- README.md

cp template/*.md docs
cp documentation/*.md docs
cp *.md docs
rm -rf documentation


cat docs/README.md > docs/home.md

docnado --html docs
git add docs
