<!--
title:      Anonymous Cloning
desc:       This article provides a technical guide for anonymous git cloning of ADORe.
date:       ${DOC_DATETIME}
version:    ${DOC_VERSION}
template:   document
nav:        Technical Documentation __4__>Anonymous Cloning __1__
percent:    100
authors:    opensource-ts@dlr.de
-->
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
* Andrew Koerner
********************************************************************************
-->
           
# Anonymous Cloning
In order to make development more friendly nearly all git submodules are 
configured to use git over ssh via the .gitmodules. The downside of this is that
github requires account keys to be configured to in order to clone the repository.
If you attempt to clone without configuring your account keys you will receive 
the following error:

```bash
git clone git@github.com:eclipse/adore.git
Cloning into 'adore'...
The authenticity of host 'github.com (140.82.121.3)' can't be established.
ECDSA key fingerprint is SHA256:p2QAMXNIC1TJYWeIOttrVc98/R1BUFWu3/LiyKgUfQM.
Are you sure you want to continue connecting (yes/no/[fingerprint])? yes
Warning: Permanently added 'github.com,140.82.121.3' (ECDSA) to the list of known hosts.
git@github.com: Permission denied (publickey).
fatal: Could not read from remote repository.

Please make sure you have the correct access rights
and the repository exists.
```

## Anonymous Cloning Over HTTPS

You can configure git to exclusively use https. This can be done with the 
following commands:

```bash
git config --global url."https://github.com/".insteadOf git@github.com:
git config --global url."https://".insteadOf git://
```

Next, you can clone the repository as normal except use https:
```bash
git clone -recurse-submodules -j$(nproc) https://github.com/eclipse/adore.git
```
