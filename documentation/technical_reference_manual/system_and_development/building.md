# Building components within ADORe
In general every module in ADORe offers the following general build related make
targets/recipes: `make help`, `make build`, `make clean`, and `make test`.
To discover what targets/recipes are provided by a given module navigate to the 
module and run: `make help`

## ADORe Core
On first invocation all of the ADORe CLI with `make cli` **only** core modules 
are built. If you are using other modules you may need to manually invoke a 
build or use the provide `make build_all` target. 


| :warning: WARNING          |
|:---------------------------|
| Only core ADORe modules are build with `make cli` |
| If you are seeing ros node errors when running roslaunch |
| try the `make build_all` target provided by adore |
| or manually build the module you are trying to use see: [Building](#Building). |



## Building 
You can manually invoke rebuilding of any ADORe module if it supports it.
1. First you have to set the SUBMODULES_PATH environmental variable.
- Navigate to the root of the ADORe project and run: `export SUBMODULES_PATH=$(pwd)`
or
- Source the provided environment file by navigating to the root of the ADORe
  project and running: `source adore.env`
2. Now you can navigate to any module and manually invoke a build
```bash
cd adore_if_ros_msg
make build
```
```bash
cd sumo_if_ros
make build
```

## build_fast targets
Several modules offer a "build_fast" target. When this target is invoked the
module is only built if it has not already been built i.e., if you run `docker
image ls` and see your module then a build will not occur. This is to save on
build time and cache invalidation. 

For theses modules such as the adore_cli and plotlabserver for example you can 
manually invoke `make build` or if you are on the root of the ADORe project
`make build_adore_cli` and `make_build_plotlabserver` to trigger a build

You can also remove the offending image from your local docker repository with
`docker image rm <image name>` and then invoke `make cli` to trigger a rebuild.

These modules do not change often and only subsequently need to be built once.
Any module with a "build_fast" target will exhibit this behavior.

