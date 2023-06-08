<!--
********************************************************************************
* Copyright (C) 2017-2020 German Aerospace Center (DLR). 
* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
*
* This program and the accompanying materials are made available under the 
* terms of the Eclipse Public License 2.0 which is available at
* http://www.eclipse.org/legal/epl-2.0.
*
* SPDX-License-Identifier: EPL-2.0 
*
* Contributors: 
*   Andrew Koerner
*   Daniel Heß 
********************************************************************************
-->

# ADORe System Requirements
- CPU: Intel CORE i7 7700K or equivalent/better
  - The more cores you have, the more trajectory planners you can run in parallel.
- No specific graphics card is required as everything (except plotting) runs on the CPU
- RAM: Min 8GB for execution. Compilation process is faster with 16+GB.
- HDD storage
  - at least 2.5 GB to clone the repository
  - at least 15 GB to build all necessary docker context
- Operating system: Anything that supports newer docker versions. 
  - Recommended: Ubuntu 20.04 or 22.04
- Network: A reliable network with high throughput and low latency.  Initial build
can take a significant amount of time to pull all necessary dependencies from apt and docker.
A poor connection will result in non-deterministic build failures. 
- Software
  - Docker v20.10.17 or greater and docker compose v2.6.0 or greater
  - Make
