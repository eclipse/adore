#!/usr/bin/env bash

function echoerr { echo "$@" >&2; exit 1;}


SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]:-$0}"; )" &> /dev/null && pwd 2> /dev/null; )";

ADORE_ROOT_DIR="$(realpath ${SCRIPT_DIR}/..)"
LOG_DIR="${ADORE_ROOT_DIR}/.log"
ROS_LOG_DIR="${LOG_DIR}/.ros/log"
PLOTLABSERVER_LOG_DIR="${LOG_DIR}/plotlabserver"


clear
cd ${SCRIPT_DIR}/.. 


bash plotlabserver/tools/wait_for_plotlab_server.sh

echo ""
printf "  Waiting for catkin workspace ..."
until [ -e catkin_workspace/install/setup.sh ]; do
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

    scenario_name=$(basename "${test_scenario}" .launch)
    mkdir -p "${LOG_DIR}/${scenario_name}/.ros"
    mkdir -p "${LOG_DIR}/${scenario_name}/plotlabserver"
    echo "Running scenario: ${test_scenario}"
    roslaunch "${test_scenario}"
    cd "${ROS_LOG_DIR}"
    latest_ros_log_path="$(ls -t | head -1)"
    rm latest -f
    ln -s -f "${latest_ros_log_path}" latest 2> /dev/null || true
    ln -s -f "${latest_ros_log_path}" "${scenario_name}" 2> /dev/null || true
    #if [[ -L latest ]]; then
    #    latest_ros_log_path="$(file latest | cut -d " " -f6)"
    #    if [[ -z "${latest_ros_log_path}" ]]; then
    #        latest_ros_log_path="$(file latest | cut -d " " -f5)"
    #    fi
    #    latest_ros_log_path_basename="$(basename "${latest_ros_log_path}")"
    #fi
    #ln -s -f "${latest_ros_log_path_basename}" latest 2> /dev/null || true
    #mv -T latest "${scenario_name}"
    mkdir -p "${LOG_DIR}/${scenario_name}/.ros"
    cp -r "${ROS_LOG_DIR}/${scenario_name}"/* "${LOG_DIR}/$scenario_name/.ros/"
    #cp -r "${ROS_LOG_DIR}/${scenario_name}/* ${LOG_DIR}/$scenario_name/.ros/
    (cd "${PLOTLABSERVER_LOG_DIR}" && cp -r * "${LOG_DIR}/${scenario_name}/plotlabserver")
    #cp -r $PLOTLABSERVER_LOG_DIR/* ${LOG_DIR}/$scenario_name/plotlabserver/
done
