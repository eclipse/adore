#!/usr/bin/env bash

# This shell script calcuates the delta between subsequent 'docker system df' calls.
#    The following goals are in place for this tool:
#        - track changes over time of the docker docker daemon disk usage
#        - use no external dependencies or tools aside from posix tools

set -e



function echoerr { echo "$@" >&2; exit 1;}
SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
iso8601_datetime="$(date -u +"%Y-%m-%dT%H:%M:%SZ")"

LOG_FILE_PREFIX="docker_storage_inventory"
LOG_FILE_EXTENSION="json"

log_file="${LOG_FILE_PREFIX}_${iso8601_datetime}.${LOG_FILE_EXTENSION}"


POSITIONAL_ARGS=()

while [[ $# -gt 0 ]]; do
  case $1 in
    -l|--log-directory)
      LOG_DIRECTORY="$2"
      shift # past argument
      shift # past value
      ;;
    *)
      POSITIONAL_ARGS+=("$1") # save positional arg
      shift # past argument
      ;;
  esac
done

set -- "${POSITIONAL_ARGS[@]}" # restore positional parameters

echo "docker_storage_inventory.sh"

if [[ -z "${LOG_DIRECTORY}" ]]; then
    echoerr "ERROR: No log directory provided. A log directory must be specified with -l or --log-directory flag."
fi

if [[ ! -d "${LOG_DIRECTORY}" ]]; then
    echoerr "ERROR: The provided log directory: ${LOG_DIRECTORY} does not exist."
fi

log_directory_absolute_path=$(realpath "${LOG_DIRECTORY}")


echo "  Using LOG_DIRECTORY: ${log_directory_absolute_path}"
echo "    log file: ${log_directory_absolute_path}/${log_file}"

log_file="${log_directory_absolute_path}/${log_file}"

echo "[" > "${log_file}"
docker system df --format "{{ json . }}," >> "${log_file}" 
echo "]" >> "${log_file}"
sed -i -z "s|,\n]|\n]|g" "${log_file}"

inventory_count="$(cd "${LOG_DIRECTORY}" && ls -1q $LOG_FILE_PREFIX* | wc -l)"
#echo "    docker storage inventory count: ${inventory_count}"


if [[ $inventory_count -lt 2 ]]; then
    echo "    Skipping delta calculation, not enough docker storage inventory."
    exit 0
fi

current_log="${LOG_DIRECTORY}/$(cd "${LOG_DIRECTORY}" && ls -Art $LOG_FILE_PREFIX* | tail -n 1)"
last_log="${LOG_DIRECTORY}/$(cd "${LOG_DIRECTORY}" && ls -Art $LOG_FILE_PREFIX* | tail -n 2 | head -n 1)"
#echo "    current log: ${current_log}"
#echo "    last log: ${last_log}"

current_log_docker_storage_size=$(cat ${current_log} | sed ':a;N;$!ba;s/\n//g' | grep -Eo '"Size"[^,]*' | grep -Eo '[^:]*$' | grep "GB" | sed "s|GB||g" | sed "s|\"||g" | awk '{s+=$1} END {print s}')
last_log_docker_storage_size=$(cat ${last_log} | sed ':a;N;$!ba;s/\n//g' | grep -Eo '"Size"[^,]*' | grep -Eo '[^:]*$' | grep "GB" | sed "s|GB||g" | sed "s|\"||g" | awk '{s+=$1} END {print s}')

#echo "    current log docker storage size: ${current_log_docker_storage_size}GB"
#echo "    last log docker storage size: ${last_log_docker_storage_size}GB"
docker_storage_size_delta=$(awk "BEGIN{ print $current_log_docker_storage_size - $last_log_docker_storage_size }")
echo "    docker storage size delta: ${docker_storage_size_delta}GB"
sed -i "s|\]||g" "${current_log}"
printf "{\"docker_storage_delta\":\"${docker_storage_size_delta}GB\", \"docker_storage_size_total\":\"${current_log_docker_storage_size}GB\"}\n" >> "${current_log}"
sed -i -r 's|}|},|g' "${current_log}" 
sed -i -r 's|,,|},|g' "${current_log}" 
sed -i -r '/^\s*$/d' "${current_log}" 
sed -i '$ s/.$//' "${current_log}"

echo "]" >> "${current_log}"
echo ""
echo "    Docker is currently using ${current_log_docker_storage_size}GB of storage. Consider clearing unneeded docker resources."
echo ""
echo "        You can use the following commands to clear docker system resources:"
echo "            'docker system prune'"
echo "            'docker docker builder prune'"
echo "        review the official docker documentaiton for more info: https://docs.docker.com/config/pruning/ "
echo ""




