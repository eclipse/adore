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
*   Daniel Heß 
********************************************************************************
-->
[https://stackoverflow.com/questions/42809088/how-to-validate-a-xml-file-with-xsd-through-xmllint](https://stackoverflow.com/questions/42809088/how-to-validate-a-xml-file-with-xsd-through-xmllint)

~~~bash
xmllint --schema ~/catkin_ws/build/adore_if_ros/_deps/xodr-src/schema/OpenDRIVE_1.4H.xsd yourfile.xodr --noout
~~~