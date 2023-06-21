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
yes | curl -sSL https://raw.githubusercontent.com/DLR-TS/adore_tools/master/tools/install_docker.sh | bash -
```
Follow the prompts until the installation completes. Once complete you should be greeted with something such as this:
```text
Hello from Docker!
This message shows that your installation appears to be working correctly.
```
> :warning: **Warning**: As a general rule you should **never** run shell scripts from untrusted sources especially as root. 

> :warning: **Warning**: After installing Docker you may need to log out and log back in to make group changes take effect. 


## Docker compose
The ADORe build system uses docker compose. At some point the docker compose
tool was renamed from "docker-compose" to "docker compose". If you have an older
version of docker you will receive an error. Please update your Docker engine.

