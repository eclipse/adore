title:      System Requirements
desc:       This guild offers a starting point for building ADORe and running a test scenario.
date:       ${DOC_DATETIME}
version:    ${DOC_VERSION}
template:   document
nav:        System Requirements __3__
percent:    100
authors:    opensource-ts@dlr.de

# ADORe System Requirements
- CPU: Intel CORE i7 7700K or equivalent/better
  - The more cores you have, the more trajectory planners you can run in parallel.
  - Building speeds up with more cores.
- No specific graphics card required as basically everything (except plotting) runs on CPU
- RAM: Min 8GB for execution. Building is better with 16+GB.
- HDD storage
  - at least 2.5 GB to clone the repository
  - at least 15 GB to build all necessary docker context
- Operating system: Anything that supports newer docker versions. 
  - Recommended: Ubuntu 20.04 or 22.04
- Software
  - Docker v20.10.17 or greater and docker compose v2.6.0 or greater
  - Make