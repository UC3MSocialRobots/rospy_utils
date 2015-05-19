rospy\_utils
============

.. image:: https://travis-ci.org/UC3MSocialRobots/rospy_utils.svg?branch=master
    :target: https://travis-ci.org/UC3MSocialRobots/rospy_utils

.. image:: https://coveralls.io/repos/UC3MSocialRobots/rospy_utils/badge.svg?branch=master
    :target: https://coveralls.io/r/UC3MSocialRobots/rospy_utils?branch=master

.. image:: https://readthedocs.org/projects/rospy-utils/badge/?version=latest
    :target: https://readthedocs.org/projects/rospy-utils/?badge=latest
    :alt: Documentation Status

A ROS package that contains several modules to work easier with rospy.

At its current version it contains 3 main modules:

-  ``param_utils``: Utilities to work with ROS parameters
-  ``func_utils``: Several generic functions
-  ``coroutines``: Utilities to enable pipelining of data processing
   methods.

Future version will include another module that ease the programming of
topic publishers and subscribers.

Installing instructions
-----------------------

To install ``rospy_utils``, you need to follow these steps:

.. code:: bash

    $ cd <your_catkin_ws_dir>/src
    $ git clone https://github.com/UC3MSocialRobots/rospy_utils.git
    $ rospack profile
    $ roscd rospy_utils
    $ # Install dependencies of the package
    $ sudo pip install -r requirements.txt

Testing
-------

You can also test it by runing ``nosetests`` or ``tox`` inside the
``rospy_utils`` dir.

Examples
--------

Comming soon

LICENSE:
--------

The license of the package is custom LASR-UC3M (Licencia Académica
Social Robotics Lab - UC3M), an open, non-commercial license which
enables you to download, modify and distribute the code as long as you
distribute the sources.

Quality Metrics
---------------

.. image:: https://travis-ci.org/UC3MSocialRobots/rospy_utils.svg?branch=master
    :target: https://travis-ci.org/UC3MSocialRobots/rospy_utils

.. image:: https://coveralls.io/repos/UC3MSocialRobots/rospy_utils/badge.svg?branch=master
    :target: https://coveralls.io/r/UC3MSocialRobots/rospy_utils?branch=master


Code Quality Reviews:
^^^^^^^^^^^^^^^^^^

I'm using different automatic code review tools since each one provides
slightly different code checks.

.. image:: https://landscape.io/github/UC3MSocialRobots/rospy_utils/master/landscape.svg?style=flat
    :target: https://landscape.io/github/UC3MSocialRobots/rospy_utils/master
    :alt: Code Health

.. image:: https://scrutinizer-ci.com/g/UC3MSocialRobots/rospy_utils/badges/quality-score.png?b=master
    :target: https://scrutinizer-ci.com/g/UC3MSocialRobots/rospy_utils/?branch=master
    :alt: Scrutinizer Code Quality

.. image:: https://codeclimate.com/github/UC3MSocialRobots/rospy_utils/badges/gpa.svg
   :target: https://codeclimate.com/github/UC3MSocialRobots/rospy_utils
   :alt: Code Climate

.. image:: https://www.codacy.com/project/badge/fa51233d02db472eaab9fb0351b40fee
    :target: https://www.codacy.com/app/vgonpa/rospy_utils
    :alt: Codacy Code Quality
