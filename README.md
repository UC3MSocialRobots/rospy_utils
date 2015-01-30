# rospy_utils
[![Build Status](https://travis-ci.org/UC3MSocialRobots/rospy_utils.svg?branch=master)](https://travis-ci.org/UC3MSocialRobots/rospy_utils)
[![Documentation Status](https://readthedocs.org/projects/rospy-utils/badge/?version=latest)](https://readthedocs.org/projects/rospy-utils/?badge=latest)


A ROS package that contains several modules to work easier with rospy.

At its current version it contains 3 main modules:

- `param_utils`: Utilities to work with ROS parameters
- `func_utils`: Several generic functions 
- `coroutines`: Utilities to enable pipelining of data processing methods.

Future version will include another module that ease the programming of topic publishers and subscribers.

## Installing instructions

To install `rospy_utils`, you need to follow these steps:

```bash
$ cd <your_catkin_ws_dir>/src
$ git clone https://github.com/UC3MSocialRobots/rospy_utils.git
$ rospack profile
$ roscd rospy_utils
$ # Install dependencies of the package
$ sudo pip install -r requirements.txt
```
You can also test it by runing `nosetests` or `tox` inside the `rospy_utils` dir.


## LICENSE:

The license of the package is custom LASR-UC3M (Licencia Académica Social Robotics Lab - UC3M), an open, non-commercial license which enables you to download, modify and distribute the code as long as you distribute the sources.  


## Quality Metrics

[![Build Status](https://travis-ci.org/UC3MSocialRobots/rospy_utils.svg?branch=master)](https://travis-ci.org/UC3MSocialRobots/rospy_utils)
[![Code Health](https://landscape.io/github/UC3MSocialRobots/rospy_utils/master/landscape.svg)](https://landscape.io/github/UC3MSocialRobots/rospy_utils/master)
[![Scrutinizer Code Quality](https://scrutinizer-ci.com/g/UC3MSocialRobots/rospy_utils/badges/quality-score.png?b=master)](https://scrutinizer-ci.com/g/UC3MSocialRobots/rospy_utils/?branch=master)
[![Coverage Status](https://coveralls.io/repos/UC3MSocialRobots/rospy_utils/badge.svg?branch=master)](https://coveralls.io/r/UC3MSocialRobots/rospy_utils?branch=master)
