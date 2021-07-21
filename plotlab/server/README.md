<!--
 *******************************************************************************
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
 * Daniel Heß
 ********************************************************************************
 -->
# plotlabserver
Plotlabserver is a tool for distributed, real-time visualization of scientific data. The server manages several OpenGL windows with 2d/3d axes. Clients send data and plot commands via TCP, to be visualized by the server. The server provides options to export graphs as Matlab or Python/matplotlib code.

## Description:
- compile by executing build.sh
- start the server by executing start.sh
- server will create 10 figures as initially hidden windows
- after startup, clients may connect to server and manipulate figures, see plotlablib/README.md for commands

## UI window manipulation:
- keyboard "m" Export Matlab file "plotfile_#date_#time_#figno_#screenshotno.m"
- keyboard "p" Export 2D-Python file "PythonPlotfile_#date_#time_#figno_#screenshotno.py"
- keyboard "o" Export 3D-Python file "PythonPlotfile_#date_#time_#figno_#screenshotno.py"
- keyboard "a" switch between projections: scaled+orthogonal vs. non-scaled+perspective (more useful for 3d)
- keyboard "l" enter/leave log mode
- keyboard "1,2,3..."" Switch between viewing angles (for 3d)
- keyboard "6" Toggle visibility of axes
- keyboard "7" Toggle grid
- drag left mouse button: rotate viewport
- double click left mouse button: reset view
- mouse wheel: zoom
- +/- zoom with keyboard
- drag right mouse button: translate

## Log mode:
While in log mode, the position of the curser is shown in the figure. Additionally, a log file is opened in the execution folder of the plotlabserver. The following actions are available in log mode:
- single click left mouse button: Write the current curser position to the log file as well as heading coming from the previously logged point
- keyboard "n" Write a line tag to the log file to seperate different coordinate groups

## Setup:
To install [dependencies](dependencies.md) in Ubuntu (18.04 or 20.04), try:
~~~bash
sudo apt-get install libzmq3-dev freeglut3-dev libcurl4-openssl-dev
~~~
Build and start Plotlabserver:
~~~bash
./build.sh
./start.sh
~~~


## License
The source code and the accompanying material is licensed under the terms of the [EPL v2](LICENSE).

## Dependencies
Plotlabserver [depends](dependencies.md) on external software packages.

## Contributing
Please see the [contribution guidelines](CONTRIBUTING.md) if you want to contribute.

## Contributors
- Daniel Heß
- Thomas Lobig
- Matthias Nichting