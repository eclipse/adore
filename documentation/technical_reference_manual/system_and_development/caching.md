# Caching
The ADORe build system contains a number of sources of data caching. Data 
caching provides several technical benefits depending on context including 
speeding up building and rebuilding, as well as, saving networking resources. 
The trade off for this is of course local disk usage. 

Cache grows over time and can also become stale requiring cleaning and pruning. 
The various sources of cache within ADORe will be discussed in the following 
sections. Caching also add complexity when fetching packages. Unreliable network
with low throughput, high latency or jitter can result in cache corruption or
non-deterministic HTTP failures with apt cacher ng. 

## APT Cacher Ng
Apt cacher ng provides caching capability for the APT package manager within 
docker containers. It saves network resources by caching/storing debian/ubuntu 
system packages so that on subsequent fetching a package is provided via local 
cache on disk verses fetching the package from a given remote APT repository.

### Cleaning APT Cacher Ng Cache
To clear/delete or clean the APT cacher ng cache you can use the provided make 
target as follows:
```bash
make clean_apt_cacher_ng_cache
```

## Docker Image Caching
Docker image caching is a feature of the ADORe build system and a component of 
the make_gadgets submodule locate at: 
[https://github.com/DLR-TS/make_gadgets](https://github.com/DLR-TS/make_gadgets)

Docker image caching supports storing/saving/caching docker images from the 
local registry to a local directory containing one tar archive per image. Docker
base images can be significantly large and fetching them from the central docker
registry can be time consuming and wasteful of network resources, as well as, 
potentially running into docker.io free quotas if repeated pulls are done from 
the same network. Docker image caching aims to mitigate this by saving base 
images locally as archives that can be dynamically loaded as needed instead of 
fetching and re-fetching them from the central registry.  

This cache is located in the root of the ADORe project in the 
'.docker_image_cache' directory'

### Cleaning Docker Image Cache
Docker image cache can be cleaned by simply deleting the 
'.docker_image_cache' directory or calling the provided make target as follows:
```bash
make clean_docker_image_cache
```

## Docker Cache and Registry
Every time docker build or docker compose build are executed on a docker context 
the docker daemon generates layer cache. This cache can grow **significantly** 
over time and thus it is important to maintain this cache.

The following are sources of docker cache:
- Volumes
- Networks
- Build/Layer cache
- Images


### Cleaning Docker Cache
The following targets are provided for cleaning various sources of docker cache:
```
docker_orbital_cannon: ## Deletes ALL docker images, volumes, build cache and containers.
docker_clean: ## Clean/delete all docker dangling images and build cache
docker_delete_all_none_tags: ## Delete all docker orphaned/none tags
docker_delete_dangling_images: ## Delete all dangling images/tags
docker_delete_all_build_cache: ## Delete all docker builder cache
docker_delete_system_prune: ## Prune the docker system
```
There is a make target provided from the make_gadgets that will clean **all** 
docker related cache. You can call this target with the following command:
```bash
make docker_orbital_cannon
```
Use this target with **caution** because it will clean the entire system of 
docker cache.
Docker cache is a deep topic and it is recommended that you review the official 
Docker documentation on the topic at: 
[https://docs.docker.com/config/pruning/](https://docs.docker.com/config/pruning/)

