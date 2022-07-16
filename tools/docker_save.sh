#!/usr/bin/env bash


SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
images=$(grep -rn "FROM " "${SCRIPT_DIRECTORY}/.." | grep Dock | cut -d " " -f2 | sort | uniq)

rm -rf docker_images
mkdir docker_images
cd docker_images

set -x

while IFS= read -r image; do
    docker pull "${image}" && docker save --output "${image//:/_}.tar" "${image}"
done <<< "$images"
docker pull edrevo/dockerfile-plus && docker save --output edrevo_dockerfile-plus.tar edrevo/dockerfile-plus
