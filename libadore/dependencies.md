<!--
#********************************************************************************
#* Copyright (C) 2017-2020 German Aerospace Center (DLR). 
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
#*
#* This program and the accompanying materials are made available under the 
#* terms of the Eclipse Public License 2.0 which is available at
#* http://www.eclipse.org/legal/epl-2.0.
#*
#* SPDX-License-Identifier: EPL-2.0 
#*
#* Contributors: 
#********************************************************************************
-->

# The following libraries will be downloaded into the external folder
- [plotlablib](https://github.com/dlr-ts/plotlablib)
- [catch2](https://github.com/catchorg/Catch2)
- [dlib](http://dlib.net/)
- [qpOASES](https://projects.coin-or.org/qpOASES)
- [xodr](https://github.com/dlr-ts/xodr)
- [xsd](https://www.codesynthesis.com/products/xsd/)
# The following libraries have to be provided by the operating system
Licenses of the following libraries may depend on the implementation installed on your system
- [xerces-c](https://xerces.apache.org/xerces-c/)
- BLAS or OpenBLAS
- LAPACK
- Boost
# Package depdencies
- CMake > 3.13 - [install as package](https://apt.kitware.com/) - reasoning for version [A](http://dominikberner.ch/cmake-interface-lib/) and [B](https://crascit.com/2016/01/31/enhanced-source-file-handling-with-target_sources/)