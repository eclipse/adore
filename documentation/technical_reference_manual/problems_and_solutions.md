

## Problem: During roslaunch some node cannot be found
You run a scenario and roslaunch reports that some node cannot be found. Not
all nodes are built by default on invocation of `make cli` resulting in some
nodes not being availabe. In this situation you could see an output similar 
to the following:

```bash
...
process[vehicle0/tlplotter-17]: started with pid [373]
ERROR: cannot launch node of type [adore_v2x_sim/channel_sim_node]: Cannot locate node of type [channel_sim_node] in package [
adore_v2x_sim]. Make sure file exists in package path and permission is set to executable (chmod +x)                          
ERROR: cannot launch node of type [adore_if_v2x/v2x_trafficlights_node]: Cannot locate node of type [v2x_trafficlights_node] i
n package [adore_if_v2x]. Make sure file exists in package path and permission is set to executable (chmod +x) 
...
```

### Solution
The fix for this is to deliberately build the node you are interested in. In the 
previous example the modules "adore_v2x_sim" and "adore_if_v2x" cannot be found.
One fix is to manually build each
1. cd to the top level of the adore project
2. source the adore.env file into your current interactive shell
```bash
source adore.env
```

Alternately, you can build all of the modules by invoking the "build_all" target
such as follows:
```bash
cd <adore project root directory>
make build_all
```

## Problem: make error when running make commands on individual modules
I am receiving a similar error message to this when running any make command
on an individual ADORe module:
```bash
INFO: To clone submodules use: 'git submodule update --init --recursive'
INFO: To specify alternative path for submodules use: SUBMODULES_PATH="<path to submodules>" make build'
adore_if_ros_msg.mk:21: *** "ERROR: adore/adore_if_ros_msg/make_gadgets does not exist. Did you clone the submodules?".  Stop.
```
---

### Solution
This is occurring because by default when cloning adore submodules are not cloned
recursively.

There are three possible solutions

1. source the provided top level adore.env file:
```bash
adore(master) ✗ (0)> source adore.env
adore(master) ✗ (0)> cd adore_if_ros_msg
adore_if_ros_msg(master) (0)> make help
Usage: make <target>
  build                                    Build adore_if_ros_msg
  clean                                    Clean adore_if_ros_msg build artifacts
  branch_adore_if_ros_msg                  Returns the current docker safe/sanitized branch for adore_if_ros_msg
...
```
2. Manually provide the SUBMODULES_PATH when invoking make on a module:
```
adore(master) ✗ (0)> cd adore_if_ros_msg
adore_if_ros_msg(master) (0)>  SUBMODULES_PATH="$(realpath ..)" make help
Usage: make <target>
  build                                    Build adore_if_ros_msg
  clean                                    Clean adore_if_ros_msg build artifacts
  branch_adore_if_ros_msg                  Returns the current docker safe/sanitized branch for adore_if_ros_msg
```
3. Recursively clone all the submodules:
```bash
adore(master) ✗ (0)> git submodule update --init --recursive
```
For more information on this please refer to the [Build System](system_and_development/build_system.md) documentation.


## Problem: Build fails when pulling apt dependencies

### Solution


## Other Problems?
Have you encountered a problem that is not documented? Create an [issue](https://github.com/eclipse/adore/issues).
