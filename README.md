rqt_mrta
=========

This is a C++ Plugin for configuring and monitoring Multi Robot Task Allocation architectures.

## Overview

**Author(s):** Adriano Henrique Rossette Leite

**Maintainer:** UNIFEI Exppertinos <expertinos DOT unifei AT gmail DOT com>

**License:** GNU Lesser General Public License (LGPL)

**Operating system(s):** Debian-based Linux

## Content

This is a C++ Plugin for configuring and monitoring Multi Robot Task Allocation architectures.

## Installation
### Dependencies

- [rqt](http://wiki.ros.org/rqt)

    ```shell
    sudo apt-get install ros-indigo-rqt
    

## Usage

To launch the standalone rqt plugin, run

```shell
rosrun rqt_mrta rqt_mrta
``````

or simply:

```shell
rqt_mrta
```

To launch the rqt GUI without a perspective, run

```shell
rm ~/.config/ros.org/rqt_gui.ini
rqt --force-discover
```

This will discover all plugins, which can then be loaded manually.

To delete the default configuration files (in case of problems):

```shell
rqt --clear-config
```
