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
********************************************************************************
-->
<!--
title:      Static Checking Tools
desc:       This article provides a technical overview of the static checking tools baked into ADORe .
date:       ${DOC_DATETIME}
version:    ${DOC_VERSION}
template:   document
nav:        Technical Documentation __4__>Static Checking Tools __2__
percent:    100
authors:    opensource-ts@dlr.de
-->           
# Static Checking Tools
There are a number of static checking tools build into ADORe intended to increase code quality.  This article will outline
the use of these tools within ADORe.

## Cppcheck

Cppcheck is a static analysis tool for C++.
More information on Cppcheck can be found at the following link [https://cppcheck.sourceforge.io/](https://cppcheck.sourceforge.io/)

### Basic Usage
Navigate to a supporting module and run the provided make target:
```bash
cd libadore
make cppcheck
```
You can run cppcheck for all supporting components by navigating to the root of the adore repository and running the provided make target:
```bash
cd adore
make cppcheck
```

### Log Report
When you run cppcheck a log report will be generated with the following name: <component name>_cppcheck_report.log
If you run make lint from the root of the adore repository all lint reports can be found in the .log directory in the root
of the repository.

## Cpplint
Cpplint is a command-line tool to check C/C++ files for style issues following Google's C++ style guide.
More information on cpplint can be found at [https://github.com/cpplint/cpplint](https://github.com/cpplint/cpplint)

### Basic Usage
Navigate to a supporting module and run the provided make target:
```bash
cd libadore
make lint
```
You can run cpplint for all supporting components by navigating to the root of the adore repository and running the provided make target:
```bash
cd adore
make lint
```

### Log Report
When you run cpplint a log report will be generated with the following name: <component name>_lint_report.log
If you run make lint from the root of the adore repository all lint reports can be found in the .log directory.

## Lizard
Lizard is a code complexity static analysis tool.
More information on lizard can be found at [https://github.com/terryyin/lizard](https://github.com/terryyin/lizard)

### Basic Usage
Navigate to a supporting module and run the provided make target:
```bash
cd libadore
make lizard
```
You can run lizard for all supporting components by navigating to the root of the adore repository and running the provided make target:
```bash
cd adore
make lizard
```

### Log Report
When you run lizard a log report will be generated with the following name: <component name>_lizard_report.xml
If you run make lint from the root of the adore repository all lint reports can be found in the .log directory.

lizard provides two log files namely, <component name>_lizard_report.xml and a human readable <component name>_lizard_report.log

