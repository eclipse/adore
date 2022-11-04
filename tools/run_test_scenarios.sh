#!/usr/bin/env bash

function echoerr { echo "$@" >&2; exit 1;}


SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]:-$0}"; )" &> /dev/null && pwd 2> /dev/null; )";

ADORE_ROOT_DIR="$(realpath ${SCRIPT_DIR}/..)"
LOG_DIR="${ADORE_ROOT_DIR}/.log"
ROS_LOG_DIR="${LOG_DIR}/.ros/log"
PLOTLABSERVER_LOG_DIR="${LOG_DIR}/plotlabserver"


clear
cd ${SCRIPT_DIR}/.. 

rm -f catkin_workspace/marker_file
make create_catkin_workspace > .log/create_catkin_workspace.log 2>&1 &
bash plotlabserver/tools/wait_for_plotlab_server.sh

echo ""
printf "  Waiting for catkin workspace ..."
until [ -e catkin_workspace/marker_file ]; do
    printf "."
    sleep 1
done
printf " done \n"
source catkin_workspace/install/setup.sh



cd adore_if_ros_demos

for test_scenario in $TEST_SCENARIOS; do
    if [[ ! -f "${test_scenario}" ]]; then
        echoerr "ERROR: Specified test scenario: ${test_scenario} is not found."
    fi

    scenario_name=$(basename $test_scenario .launch)
    mkdir -p $LOG_DIR/$scenario_name/{plotlabserver,.ros}
    echo "Running scenario: ${test_scenario}"
    roslaunch $test_scenario 
    (cd "${ROS_LOG_DIR}" && ln -s -f $(basename $(file latest | cut -d" " -f6)) latest 2> /dev/null || true)
    (cd "${ROS_LOG_DIR}" && mv -T latest $scenario_name)
    cp -r $ROS_LOG_DIR/${scenario_name}/* ${LOG_DIR}/$scenario_name/.ros/
    cp -r $PLOTLABSERVER_LOG_DIR/* ${LOG_DIR}/$scenario_name/plotlabserver/
done
