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

# Core libraries of the adore framework

libadore is a collection of libraries for education and research on Cooperative Automated Vehicles (CAV). Its goal is to support different automation levels and different driving capabilities with a set of well defined interfaces, data models and controller implementations. We strive to minimize controller complexity and the amount of required data by making use of abstractions. The library is developed system independently in c++. In order to test and simulate concrete instances of automation designs, a generic interface to middleware software / data transmission is provided. 

* [**adore/fun**](adore/fun) - motion planning, decision making and control for AV
* [**adore/env**](adore/env) - environment description, data representation for AV
* [**adore/view**](adore/view) - data abstraction interface for AV, decouples fun and env
* [**adore/mad**](adore/mad) - math, algorithms and abstract data structures supporting AV
* [**adore/apps**](adore/apps) - middleware independent implementation of adore applications
* [**adore/params**](adore/params) - abstract parameter descriptions for AV
* [**adore/sim**](adore/sim) - simulation tools
* [**adore/if_xodr**](adore/if_xodr) - conversion of OpenDrive tracks into internal representation
* [**adore/if_r2s**](adore/if_r2s) - conversion of Road2Simulation tracks into internal representation


## License
The source code and the accompanying material is licensed under the terms of the [EPL v2](LICENSE).

## Dependencies
libadore [depends](dependencies.md) on external software packages.

## Contributing
Please see the [contribution guidelines](CONTRIBUTING.md) if you want to contribute.

## Contributors
- Daniel Heß
- Stephan Lapoehn
- Thomas Lobig
- Matthias Nichting
- Robert Markowski
- Jan Lauermann
- Reza Deriani
- Jonas Rieck
