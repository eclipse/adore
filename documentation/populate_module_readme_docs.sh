#!/usr/bin/env bash

set -euo pipefail
#set -euxo pipefail #debug mode
#
echoerr (){ printf "%s" "$@" >&2;}
exiterr (){ printf "%s\n" "$@" >&2; exit 1;}

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

SUBMODULES_PATH="$(realpath "${SCRIPT_DIRECTORY}/..")"

EXCLUDES="documentation external/ros-bridge"

DOCS_DIR="${SCRIPT_DIRECTORY}/technical_reference_manual"
MODULE_TEMPLATE_MD_FILE="${SCRIPT_DIRECTORY}/modules.md"
MODULE_DOCS_DIR="${DOCS_DIR}/modules"

rm -rf "${MODULE_DOCS_DIR}"

echo "SUBMODULES_PATH: ${SUBMODULES_PATH}"

module_readme_files="$(find "${SUBMODULES_PATH}" -type f -name README.md | sort)"

for exclude in $EXCLUDES; do
    module_readme_files="$(echo "${module_readme_files}" | grep -v "${exclude}")"
done


#echo "${module_readme_files}"
module_md=$(< "${MODULE_TEMPLATE_MD_FILE}")
url=
last_module=""
for module_readme_file in $module_readme_files; do
    if [[ ! -s "$module_readme_file" ]]; then
       continue 
    fi

    #cp "${module_readme_file}" "${MODULE_DOC_DIR}"
    module_readme_absolute_path=$(dirname "${module_readme_file}")
    cd "${module_readme_absolute_path}" 
    module_absolute_path="$(git rev-parse --show-toplevel)"
    remote="$(git config --get remote.origin.url)"
    url=$(echo ${remote/git@github.com:/https:\/\/github.com\/} | sed 's/\.git$//')
    module="$(basename -s .git "$remote")"
    echo " module_readme_file: '${module_readme_file}'"
    module_readme_relative_path="${module_readme_file/$module_absolute_path/}"
    module_readme_relative_path=""${module_readme_relative_path/\//}""
    module_readme_relative_path="${module_readme_relative_path/README.md/}"
    module_readme_relative_path="${module_readme_relative_path%/}"
    module_readme_relative_path="${module_readme_relative_path%/}"
    module_readme_relative_path="${module}/${module_readme_relative_path}"
    module_readme_relative_path="${module_readme_relative_path%/}"
    echo " module_readme_relative_path: '${module_readme_relative_path}'"

    module_readme_markdown_link="[${module_readme_relative_path}/README.md 🔗](modules/${module_readme_relative_path}/README.md)"
    mkdir -p "${MODULE_DOCS_DIR}/${module_readme_relative_path}"
    cp "${module_readme_file}" "${MODULE_DOCS_DIR}/${module_readme_relative_path}/"
    #printf "module: ${module} remote: ${remote}\n"
    if [ "$last_module" != "$module" ]; then
        last_module=$module
        echo "module: ${module}"
        echo "  remote: ${remote}"
        echo "  url: ${url}"
        module_md+=$(printf "\n\n### Module: **%s**\n\n" "${module}")
        module_md+=$(printf "\n\n**git remote**: ${remote}\n\n")
        module_md+=$(printf "\n\n**url**: [%s 🔗](%s)\n\n" "${url}" "${url}")
        module_md+=$(printf "\n\n#### %s README Links\n\n" "${module}")
    fi
    echo "    module_readme_relative_path: ${module_readme_relative_path}"
    echo "    module_readme_absolute_path: ${module_readme_absolute_path}"
    echo "    module_readme_file: ${module_readme_file}"
    echo "    module_readme_markdown_link: ${module_readme_markdown_link}"
    module_md+=$(printf "\n\n%s\n\n" "${module_readme_markdown_link}")
done

echo -n "${module_md}" > "${DOCS_DIR}/modules.md" 
