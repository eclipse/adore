#!/usr/bin/env #!/usr/bin/env bash


echo "ERROR: experimental." 1>&2
exit 1

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd "${SCRIPT_DIRECTORY}"/..

find . -type d -name make_gadgets -or -name lizard_docker -or -name cpplint_docker -or -name cppcheck_docker -or -name adore_if_ros_msg -or -name v2x_if_ros_msg -or -name coordinate_conversion | grep -v "/build/\|.git\|catkin_workspace" | while read line ; do (cd $line && pwd && git checkout master && git pull); done



