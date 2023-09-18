# Disable Apt Cacher Ng
This guide will detail how to disable apt cacher ng when it is causing issues.

## Backgound
There are some instances where it may be desirable to disable apt cacher ng.
Although apt-cacher ng can provide a number of benefits it can also cause build
failures on unreliable network connections.

## `APT_CACHER_NG_ENABLED` Environmental Variable
To disable apt cacher ng the environmental variable `APT_CACHER_NG_ENABLED` must
be set and equal to `false`. This can be done several ways.  
One-off disabling apt cacher:
```bash
APT_CACHER_NG_ENABLED=false make build
```
persistent disabling apt cacher for current interactive session:
```bash
export APT_CACHER_NG_ENABLED=false 
...
make build
```

## adore.env
There is an environmental file `adore.env` located in the root of the adore 
project.  By default the environmental `APT_CACHER_NG_ENABLED` is set to `true`
in this file.  To disable apt cacher ng edit this file and replace true with 
false. After the file has been edited you can source the environmental file:
```bash
cd adore
source adore.env
```


For more information please review the apt-cacher-ng docker documentation: 
[https://github.com/DLR-TS/apt_cacher_ng_docker](https://github.com/DLR-TS/apt_cacher_ng_docker)

