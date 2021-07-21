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
********************************************************************************
-->

# The following libraries will be downloaded automatically during build
- [catch2](https://github.com/catchorg/Catch2), license: [Boost v1.0](https://github.com/catchorg/Catch2/blob/master/LICENSE.txt)
- [dlib](http://dlib.net/), license: [Boost v1.0](http://dlib.net/license.html)
- [qpOASES](https://projects.coin-or.org/qpOASES), license: [LGPL v2.1](http://www.gnu.org/licenses/lgpl-2.1.html)
- [xodr](https://github.com/dlr-ts/xodr), license: Apache 2.0, depends on xsd
- [xsd](https://www.codesynthesis.com/products/xsd/), license: [GPL v2 with FLOSS exception](https://www.codesynthesis.com/products/xsd/license.xhtml)
- [ZeroMQ, libzmq](https://github.com/zeromq/libzmq), license: [LGPL v3](https://www.gnu.de/documents/lgpl-3.0.de.html)
- [stb_image](https://github.com/nothings/stb/blob/master/stb_image.h), license: MIT
- [csaps](https://github.com/espdev/csaps), forked to [csaps-fork](https://github.com/tlobig/csaps-cpp), license: MIT
# The following libraries have to be provided by the operating system (see install instructions)
Licenses of the following libraries may depend on the implementation installed on your system
- [ROS](http://wiki.ros.org/melodic/)
- [xerces-c](https://xerces.apache.org/xerces-c/), license: [Apache 2.0](https://github.com/apache/xerces-c/blob/trunk/LICENSE)
- BLAS or OpenBLAS
- LAPACK
- Boost
- OpenGL 
- GLUT or freeGLUT
- CURL
- pthreads
## Package depdencies
- CMake > 3.13 - [install as package](https://apt.kitware.com/) - reasoning for version [A](http://dominikberner.ch/cmake-interface-lib/) and [B](https://crascit.com/2016/01/31/enhanced-source-file-handling-with-target_sources/)
