#!/usr/bin/env bash 


DOCKER_IMAGE_DIRECTORY="docker_images"
echo "docker_load.sh:"

if [[ ! -d "${DOCKER_IMAGE_DIRECTORY}" ]]; then
  echo "  docker image directory: ${DOCKER_IMAGE_DIRECTORY} does not exist. Nothing to load skipping."
  exit 0
fi

DOCKER_IMAGE_DIRECTORY="$(realpath docker_images)"
docker_image_archives="$(ls docker_images/*.tar)"
echo "  loading saved docker images in: ${docker_image_archives}"
for docker_image in $docker_image_archives; do
  docker_image="$(realpath "${docker_image}")"
  echo "    Loading: $docker_image"
  docker load --input "${docker_image}"
done


