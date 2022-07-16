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
#*   Thomas Lobig
#********************************************************************************
-->
# Overview
libadore bundles several packages: Four central packages of the library are ENV for environment models, FUN for planner and controller implementations, VIEW for abstraction and decoupling of environment models and APPS for definition of applications based on a combination of ENV, FUN and VIEW. The package SIM defines models for the simulation of one or more automated vehicles. The package PARAMS lists sets of parameters required for vehicle automation. The package MAD defines a useful toolset for mathematical operations, algorithms and data structures. The package if_xodr converts [OpenDrive](http://www.opendrive.org/) road models to an internal road-map model.

One important design principle we strive to uphold, is to avoid direct dependencies between models and controllers/planners. The VIEW package therefore defines several task specific data abstraction interfaces. A set of models from ENV is used to provide data to a view, while a controller uses the high-level data provided by the view, without knowing the underlying models. This allows us to easily exchange, re-organize and extend environment models. Furthermore, several alternative controller implementations for a specific task can be compared without implementing a task specific data refinement multiple times.

Another important principle is to avoid depending on specific communication or middleware frameworks.
The APPS package defines several "proto" processes, which interconnect model data akquisition, abstraction/refinement and planning and control modules, without referencing concrete middleware services. 
Communication services are decoupled using an [abstract factory pattern](https://en.wikipedia.org/wiki/Factory_method_pattern). Several packages (ENV, FUN, SIM) define abstract factories to standardize their data exchange.
A separate project [adore_if_ros](../../adore_if_ros) implements middleware-dependent concrete factory and concrete product implementations.

Currently, adore_LIBS does not contain packages for sensor data fusion or interfacing with physical sensors. 
Assuming a typical "sense-plan-act" architecture, all vehicle automation modules downstream of "sense" are in the scope of this project.

# Software Documentation
Doxygen may be used to create html-based documentation in [../build/docs_html](adore_libs/build/docs_html) by executing [generate_docs.bash](generate_docs.bash).
Please use the style described [here](docstyle.md) in source code comments.
