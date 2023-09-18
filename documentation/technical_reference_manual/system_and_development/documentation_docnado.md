# Documentation
ADORe provides several sources of Documentation which will be detailed below.

## Landing page

Location: [https://eclipse.github.io/adore/w/generated_doxygen_documentation/index.html](https://eclipse.github.io/adore/w/generated_doxygen_documentation/index.html)

## Docnado
ADORe uses [Docnado](https://github.com/HEInventions/docnado) as a documentation
layout tool.

Location: [https://eclipse.github.io/adore/w](https://eclipse.github.io/adore/w)

## Doxygen
Location: [https://eclipse.github.io/adore/w/generated_doxygen_documentation/index.html](https://eclipse.github.io/adore/w/generated_doxygen_documentation/index.html)

## GNU Make
Every ADORe module provides a Makefile providing "documentation as code". To
learn what a module offers inspect the available make targets. Every ADORe
module also offers a `make help` target. Call 'make help' to learn what it
offers such as with the following example:
```bash
adore(master) ✗ (0)> source adore.env
adore(master) ✗ (0)> cd adore_if_ros_msg
adore_if_ros_msg(master) (0)> make help
Usage: make <target>
  build                                    Build adore_if_ros_msg
  clean                                    Clean adore_if_ros_msg build artifacts
  branch_adore_if_ros_msg                  Returns the current docker safe/sanitized branch for adore_if_ros_msg
  build_adore_if_ros_msg                   Build adore_if_ros_msg
  clean_adore_if_ros_msg                   Clean adore_if_ros_msg build artifacts
  docker_clean                             Clean/delete all docker dangling images and build cache
  docker_delete_all_build_cache            Delete all docker builder cache
  docker_delete_all_containers             Stop and delete all docker containers
  docker_delete_all_none_tags              Delete all docker orphaned/none tags
  docker_delete_dangling_images            Delete all dangling images/tags
  docker_orbital_cannon                    Deletes ALL docker images, volumes, build cache and containers. !DangerZone!
  docker_system_prune                      Prune the docker system
  get_sanitized_branch_name                Returns a sanitized git branch name with only alphanumeric and ASCII characters permitted as docker tags
  image_adore_if_ros_msg                   Returns the current docker image name for adore_if_ros_msg
```

---
**NOTE**
Be sure to source the adore.env file before running any make commands on
individual modules. Otherwise you could be greeted with the following error
message or similar:
```bash
INFO: To clone submodules use: 'git submodule update --init --recursive'
INFO: To specify alternative path for submodules use: SUBMODULES_PATH="<path to submodules>" make build'
adore_if_ros_msg.mk:21: *** "ERROR: adore/adore_if_ros_msg/make_gadgets does not exist. Did you clone the submodules?".  Stop.
```
---

## Documentation Generation
For information on how the sausage (documentation) is made please visit the
[Documentation Generation](documentation_generation.md)
guide.
