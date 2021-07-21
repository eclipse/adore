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
*  Daniel Heß
********************************************************************************
-->
# plotlablib

A library for the interaction with plotlabserver.

## Usage
- Create a factory object:      DLR_TS::PlotLab::FigureStubFactory figureFactory;
- Create a figure object:        DLR_TS::PlotLab::FigureStubZMQ* figure = figureFactory->createFigureStub(1); //parameter is window id
- Plot a line-strip:                   figure->plot("#myline1",X,Y,Z,N,"LineColor=1,0,0;LineWidth=1");
    - "#myline1" is a hashtag for the plotted line. Line can be later modified or deleted using the hashtag.
    - X,Y,Z are float or double arrays, e.g. as in float* X, float* Y,...
    - You can just leave away Z, if plotting 2d data
    - N is the number of points
    - "LineColor=1,0,0;LineWidth=1" is the option string: separate options with ";", separate name and value with "="
        - Supported options are FillColor=r,g,b; LineWidth=w; LineColor=r,g,b; LineStyle=none; PointSize=w;
        - Currently points are only displayed, if the LineStyle is none 
- Plot a polygon:                       figure->patch("#mypoly",X,Y,Z,N,"style");
- Append data to a line-strip:          figure->append("#myline1",X,Y,Z,N,"style");
    - Adds the supplied lines/points to the end of a existing "#myline1" object. If no "#myline1" object exists, a new one is created.
    - There is a maximum number of points to a line-strip. If this number is exceeded by appending, then points are removed at the beginning of the line strip. Thus the line strip behaves like a FIFO queue.
- Display text:                         figure->plotText(X,Y,"the text");
- Plot a texture-mapped rectangle:      figure->plotTexture("#mytexture",url,x,y,z,psi,w,l)
    - url is the an url to a https or file address of a png file
    - x, y, z give the coordinates of the center of the texture-mapped rectangle in the figure
    - psi is the orientation of the rectangle
    - w and l are widht and length of the rectangle
- Clear a figure:                       figure->clear();
- Remove a single plot:                 figure->erase("#myline1");
- Other commands for windows are: hide, show, hideAxis, showAxis, hideGrid, showGrid, setX/Y/ZLabel, setName, setTitle
- Manipulate the viewport of the figure: setViewPortOffsets

## License
The source code and the accompanying material is licensed under the terms of the [EPL v2](LICENSE).

## Dependencies
Plotlablib [depends](dependencies.md) on external software packages.

## Contributing
Please see the [contribution guidelines](CONTRIBUTING.md) if you want to contribute.

## Contributors
- Daniel Heß
- Thomas Lobig