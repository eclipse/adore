# Installing Docker 

The ADORe build system sometimes uses experimental features of docker. 
For this reason it is important to have a recent version of docker installed. 
Using "apt-get" or "apt" to install Docker is not sufficient. Software packages
within the Ubuntu central repositories are always very old.

## Installing Docker: Ubuntu 20.04 and 22.04
To install the latest version of docker in Ubuntu 20.04 and 22.04 you can follow
the official Docker document at: https://docs.docker.com/engine/install/ubuntu/

Alternatively, ADORe provides a shell script that you can use to install the latest Docker. 
```bash
curl -sSL https://raw.githubusercontent.com/DLR-TS/adore_tools/master/tools/install_docker.sh | bash -
"
```
> :warning: **Warning**: As a general rule you should **never** run shell
> scripts from untrusted sources especially as root. 



## Docker compose
The ADORe build system uses docker compose. At some point the docker compose
tool was renamed from "docker-compose" to "docker compose". If you have an older
version of docker you will receive an error. Please update your Docker engine.

