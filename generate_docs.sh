#!/usr/bin/env bash

function echoerr { echo "$@" >&2; exit 1;}
SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

rm -rf docs || true
mkdir docs

source config.env
git pull origin $GIT_BRANCH
git checkout $GIT_BRANCH -- documentation

for docfile in $(find documentation -name "**.md"); do
    docfile_basename=$(basename $docfile)
    envsubst < $docfile > docs/$docfile_basename
done

cd docs && ln -s Home.md home.md

cd "${SCRIPT_DIRECTORY}"
docker build -t docnado:latest .
docker run --user $(id -u):$(id -g) -v ${SCRIPT_DIRECTORY}/docs:/tmp/docs docnado:latest


cd docs/w && ln -s home.html index.html

cd "${SCRIPT_DIRECTORY}"
rm -rf documentation
git add docs/*

git restore --source=HEAD --staged --worktree -- documentation
