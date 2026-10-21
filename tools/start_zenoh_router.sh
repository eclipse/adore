#!/usr/bin/env bash
SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SOURCE_DIRECTORY="$(realpath "${SCRIPT_DIRECTORY}/..")"

source "${SOURCE_DIRECTORY}/adore.env" 2>/dev/null || true
source "/opt/ros/${ROS_DISTRO}/setup.bash" 2>/dev/null || true

LOG_DIR="${SOURCE_DIRECTORY}/.log/zenoh"
PIDFILE="${LOG_DIR}/zenoh_router.pid"
LOGFILE="${LOG_DIR}/zenoh_router.log"

mkdir -p "${LOG_DIR}"

if [ "${ZENOH_ROUTER_ENABLE:-false}" != "true" ]; then
    exit 0
fi

# pid can be an unrealted process
if [ -f "${PIDFILE}" ]; then
    ROUTER_PID="$(cat "${PIDFILE}")"
    case "$(cat "/proc/${ROUTER_PID}/comm" 2>/dev/null)" in
        zenohd|rmw_zenohd)
            echo "✓ Zenoh router already running (pid ${ROUTER_PID})"
            exit 0
            ;;
    esac
    echo "Ignoring stale pid file ${PIDFILE}"
    rm -f "${PIDFILE}"
fi

ROUTER_ARGS=""
if [ -f "${ZENOH_ROUTER_CONFIG:-}" ]; then
    ROUTER_ARGS="--config ${ZENOH_ROUTER_CONFIG}"
fi

ZENOHD_BIN="$(PATH="/usr/bin:/usr/local/bin:${PATH}" command -v zenohd 2>/dev/null || true)"

if [ -n "${ZENOHD_BIN}" ]; then
    echo "Starting zenoh router (zenohd) -> ${LOGFILE}"
    "${ZENOHD_BIN}" ${ROUTER_ARGS} >> "${LOGFILE}" 2>&1 &
elif command -v ros2 &>/dev/null && ros2 pkg list 2>/dev/null | grep -q rmw_zenoh_cpp; then
    echo "Starting zenoh router (rmw_zenohd) -> ${LOGFILE}"
    ros2 run rmw_zenoh_cpp rmw_zenohd ${ROUTER_ARGS} >> "${LOGFILE}" 2>&1 &
else
    echo "ERROR: no zenoh router found. Install zenohd or ros-${ROS_DISTRO}-rmw-zenoh-cpp." >&2
    exit 1
fi

echo $! > "${PIDFILE}"
echo "  pid $!"
