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
*  Thomas Lobig
********************************************************************************
-->


# Git Guide

This guide has the purpose to explain the basic Git workflow we want to use. If you don't know what Git is or have never worked with it, it is advisable to learn the basics of Git first to understand the rest of this document. Some good learning resources for Git:

* [the Git tutorial at backlog.com](https://backlog.com/git-tutorial/)
* [the interactive branching tutorial at learngitbranching.js.org](http://learngitbranching.js.org/)
* [Youtube search](https://www.youtube.com/results?search_query=git+tutorial)

> ## TODO
>
> * [ ] finish structure of this document
> * [ ] finish each part

## Setup

The setup of your environment is specific to this repository. It is assumed that git and vscode are already installed as part of the installation setup

* git package
* git config
  * certificate
  * name + email
  * commit message



~~~batch
git config --system http.sslbackend openssl
~~~

## Workflow

### Clone and Pull

### Commit and Push

### Commit in Visual Studio Code

### Moving files correctly

Don't manually move or rename files, use the git mv command instead. The following example will move and rename a file:

~~~bash
git mv folderA/fooClass.cpp folderB/foo_class.cpp
~~~

## Workaround on self signed certificates

Historically the repository was hosted on a server which used a self signed certificate. To work around a self signed certificate, download the certificate through your browser.

Short version: Click on the security information on the address bar, get additional info(click) to view the certificate (click). On the details tab you can export the certificat (click). Thus you can edit your ~/.gitconfig to include:

```ini
[http "https://xx.xx.xx.xx"]
    sslCAInfo = ~/path/to/saved/Gitlab.crt
```

* In Windows you need to choose the openssl backend for this option to work