#!/usr/bin/env bash

function exiterr { printf "$@\n" >&2; exit 1;}
SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

DOCUMENTATION_DIRECTORY="$(realpath "${SCRIPT_DIRECTORY}/../technical_reference_manual")"
DOCNADO_DIRECTORY="${SCRIPT_DIRECTORY}/docnado_templates"
DOCNADO_PREPROCESS_DIRECTORY="${SCRIPT_DIRECTORY}/html"


if ! [[ -x $(command -v "docnado") ]]; then
    exiterr "ERROR: docnado is not installed. Install with: 'python3 -m pip install docnado --upgrade'; visit https://heinventions.github.io/docnado-site/ for more info."
fi

echo "Documentation directory: ${DOCUMENTATION_DIRECTORY}"
echo "Docnado directory: ${DOCNADO_DIRECTORY}"

rm -rf "${DOCNADO_PREPROCESS_DIRECTORY}"
mkdir -p "${DOCNADO_PREPROCESS_DIRECTORY}"
cp -r $DOCUMENTATION_DIRECTORY/* "${DOCNADO_PREPROCESS_DIRECTORY}/"

source docnado.env
documentation=$(find "${DOCUMENTATION_DIRECTORY}" -name "**.md")


set -a  # or: set -o allexport 
while IFS= read -r document; do
    document_filename="${document}"
    document_base_filename="${document#"$DOCUMENTATION_DIRECTORY/"}"
    document_variable_name="${document_base_filename//\//_}"
    document_variable_name="${document_variable_name//./_}"
    document_variable_content=$(<"${document_filename}")
    declare "$document_variable_name=$document_variable_content"
done <<< "$documentation"


docnado_documentation=$(find "${DOCNADO_DIRECTORY}" -name "**.md")

while IFS= read -r document; do
    document_filename="${document}"
    document_base_filename="${document#"$DOCNADO_DIRECTORY/"}"
    echo "${document_base_filename}"
    mkdir -p "$(dirname "${DOCNADO_PREPROCESS_DIRECTORY}/${document_base_filename}")" && touch "${DOCNADO_PREPROCESS_DIRECTORY}/${document_base_filename}"
    envsubst < "${DOCNADO_DIRECTORY}/${document_base_filename}" >> "${DOCNADO_PREPROCESS_DIRECTORY}/${document_base_filename}"
    #
done <<< "$docnado_documentation"
set +a

docker build -f Dockerfile.docnado -t docnado:latest .
docker run --user $(id -u):$(id -g) -v ${DOCNADO_PREPROCESS_DIRECTORY}:/tmp/docs docnado:latest

cd "${DOCNADO_PREPROCESS_DIRECTORY}"
rm -rf index.html
ln -sf w/Home.html index.html
