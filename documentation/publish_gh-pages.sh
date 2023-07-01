#!/usr/bin/env bash

set -euo pipefail

exiterr (){ printf "$@\n"; exit 1;}


trap cleanup EXIT

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROHIBITED_REMOTES="git@github.com:eclipse/adore.git https://github.com/eclipse/adore.git"
DOCS_DIRECTORY="${SCRIPT_DIRECTORY}/docs"

printf "Creating temporary workspace directory...\n"
CLONE_DIRECTORY="$(mktemp -d)"


git_url="$(git remote get-url origin)"

function cleanup {
  rm -rf "${CLONE_DIRECTORY}"
  echo "Deleted temp working directory ${CLONE_DIRECTORY}"
}

if [[ ! -d "${DOCS_DIRECTORY}" ]]; then
    exiterr "ERROR: docs directory: ${DOCS_DIRECTORY} does not exist.  Did you call 'make build_gh-pages'" 
fi


if [[ $PROHIBITED_REMOTES == *"$git_url"* ]]; then
    exiterr "ERROR: Prohibited to publish to origin: ${git_url}, create a fork or change the origin and try again."
fi

source publish.env

read -p "Should the remote publish branch: ${PUBLISH_BRANCH} be deleted? (y/n): " answer
if ! [[ $answer == [Yy] ]]; then
    git push origin --delete "${PUBLISH_BRANCH}"
fi

cd "${CLONE_DIRECTORY}"
printf "Cloning adore to: ${CLONE_DIRECTORY}/adore\n"
git clone --depth 1 --single-branch "${git_url}"
cd adore
git branch "${PUBLISH_BRANCH}"
git checkout "${PUBLISH_BRANCH}"
git rm -rf .
git clean -fxd

cp -r "${DOCS_DIRECTORY}" "${CLONE_DIRECTORY}/adore/docs"

git add docs
git commit -am "${PUBLISH_COMMIT_MESSAGE}"

git push --force --set-upstream origin "${PUBLISH_BRANCH}" | true
git push origin "${PUBLISH_BRANCH}" 
