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

<div style="display: inline-block;">
    <div style="display: flex; align-items: center; justify-content: center;padding: 5px; border: 1px solid #FF7900; background-color: #FF7900;" >
      <strong><i class="fa fa-exclamation-triangle"></i> Warning</strong>
    </div>
    <div style="background-color: white; border: 1px solid 	#FF7900; padding: 10px;">
        <p>As a general rule you should never run shell scripts from untrusted sources especially as root.</p>
    </div>
</div><br />


<div style="display: inline-block;">
    <div style="display: flex; align-items: center; justify-content: center;padding: 5px; border: 1px solid #FF7900; background-color: #FF7900;" >
      <strong><i class="fa fa-exclamation-triangle"></i> Warning</strong>
    </div>
    <div style="background-color: white; border: 1px solid 	#FF7900; padding: 10px;">
        <p>After installing Docker you may need to log out and log back in to make group changes take effect.</p>
    </div>
</div><br />

### Verifying your Docker group
In order for you to be able to run any docker commands your current user must
be a member of the docker group. This occurs during installation but will not 
take effect until you log out and log back in again. To verify you are a member
of the docker group run: `id | sed "s|,|\n|g" | grep docker`

Which should yield something similar to:
```bash
998(docker)
```

### Verifying your docker installation
At any time you can verify your docker installation by running the docker hello
world image with the following:
```bash
docker run hello-world
```
Which will yield:
```bash
Hello from Docker!
This message shows that your installation appears to be working correctly.
```

## Docker compose
The ADORe build system uses docker compose. At some point the docker compose
tool was renamed from "docker-compose" to "docker compose". If you have an older
version of docker you will receive an error. Please update your Docker engine.

