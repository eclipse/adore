import os
import yaml

def generate_nav_config(directory):
    nav_config = []
    for filename in os.listdir(directory):
        if filename.endswith(".md"):
            file_path = os.path.join(directory, filename)
            nav_config.append(filename[:-3] + ": " + file_path)
    return nav_config

docs_directory = "docs"
nav_config = generate_nav_config(docs_directory)

config = {
    "site_name": "ADORe Documentation",
    "nav": nav_config,
}

with open("mkdocs.yml", "w") as config_file:
    yaml.dump(config, config_file)
