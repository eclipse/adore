# Problems and Solutions
This section will offer solutions to known issues.


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

### Solution: During roslaunch some node cannot be found
This likely means the node you are trying to use has not been built.

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

> **ℹ️ INFO**
>
> By default only **core** ADORe modules are built. Invoke the `make build_all`
> target from the root ADORe directory to build every module.


### Problem: make error when running make commands on individual modules
I am receiving a similar error message to this when running any make command
on an individual ADORe module:
```bash
INFO: To clone submodules use: 'git submodule update --init --recursive'
INFO: To specify alternative path for submodules use: SUBMODULES_PATH="<path to submodules>" make build'
adore_if_ros_msg.mk:21: *** "ERROR: adore/adore_if_ros_msg/make_gadgets does not exist. Did you clone the submodules?".  Stop.
```
---

### Solution: make error when running make commands on individual modules
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
For more information on this please refer to the [Build System 🔗](system_and_development/build_system.md) documentation.


### Problem: Build fails when pulling apt dependencies
During an initial build there is a significant amount of data that is pulled and 
cached from the internet. In order to lessen this burden the tool AptCacherNg is
used. This added complexity has a drawback if run on an unreliable network 
resulting in non-deterministic HTTP errors and corrupted apt packages and build
failures.

For more information on general caching in adore review the
[Caching 🔗](system_and_development/caching.md)
documentation and for more information on how AptCacherNg works visit the
project repository: [https://github.com/DLR-TS/apt_cacher_ng_docker 🔗](https://github.com/DLR-TS/apt_cacher_ng_docker)

After running `make build` you receive an HTTP error from APT such as the
following error message:

```bash
...
#0 6.829 Get:76 http://archive.ubuntu.com/ubuntu focal/main amd64 g++ amd64 4:9.3.0-1ubuntu2 [1604 B]
#0 6.833 E: Failed to fetch http://archive.ubuntu.com/ubuntu/pool/universe/a/asciidoc/asciidoc-common_9.0.0~rc1-1_all.deb  503  Resource temporarily unavailable [IP: 127.0.0.1 3142]
...
```
or
```bash
...
=> ERROR [libadore internal] load metadata for docker.io/library/alpine:3.14
=> [libadore internal] load metadata for docker.io/library/libadore_build:1d0db8b
------
 > [libadore internal] load metadata for docker.io/library/alpine:3.14
------
failed to solve: alpine:3.14: failed to do request: Head "https://registry-1.docker.io/v2/library/alpine/manifests/3.14": dial tcp: lookup registry-1.docker.io on 127.0.0.53:53: server misbehaving
```


### Solution: Build fails when pulling apt dependencies
The following section will offer a few potential solution steps you can take
to get past this issue.


#### Brute force
Simply rerunning the build command can resolve the issue. Rerun the build or make
command again, if the issue was intermittent it will continue at the previous 
failure point.

#### Clear apt cacher broken packages
On unreliable connections apt cacher packages can become corrupt. This can cause
build errors.

Delete corrupted packages with the apt cacher web interface:

1. navigate to http://127.0.0.1:3142/acng-report.html in your browser 
2. Check the: "Validate by file name AND file directory (use with care)," and "then validate file contents through checksum (SLOW), also detecting corrupt files," check boxes.
3. Then click the "Start Scan and/or Expiration" button.
4. Then click "Check all" button. 
5. Then click "Delete selected files" button followed by "Delete now" button and close the web browser.
6. Rerun the make build command.


##### Disable apt cacher entirely
APT Cacher Ng works with docker by acting as a proxy and handling any request by
apt within the docker engine.  This is accomplished via the `DOCKER_CONFIG`
environmental variable which by default is not set.  When invoking docker via
make, as with the ADORe build system this environmental variable is set to point
to the AptCacherNg service. To disable AptCacherNg use the following syntax to 
disable it: 

 - Disable AptCacherNg for one command:
```bash
DOCKER_CONFIG= make <target>
```

- Disable AptCacherNg for current interactive shell session:
```bash
export DOCKER_CONFIG= 
```


### Do You Have Other Problems?
Have you encountered a problem that is not documented? Create an [issue 🔗](https://github.com/eclipse/adore/issues).
