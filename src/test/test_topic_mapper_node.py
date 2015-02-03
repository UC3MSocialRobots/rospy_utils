#!/usr/bin/env python


# :version:      0.1.0
# :copyright:    Copyright (C) 2014 Universidad Carlos III de Madrid.
#                Todos los derechos reservados.
# :license       LASR_UC3M v1.0, ver LICENCIA.txt

# Este programa es software libre: puede redistribuirlo y/o modificarlo
# bajo los terminos de la Licencia Academica Social Robotics Lab - UC3M
# publicada por la Universidad Carlos III de Madrid, tanto en su version 1.0
# como en una version posterior.

# Este programa se distribuye con la intencion de que sea util,
# pero SIN NINGUNA GARANTIA. Para mas detalles, consulte la
# Licencia Academica Social Robotics Lab - UC3M version 1.0 o posterior.

# Usted ha recibido una copia de la Licencia Academica Social
# Robotics Lab - UC3M en el fichero LICENCIA.txt, que tambien se encuentra
# disponible en <URL a la LASR_UC3Mv1.0>.


PKG = 'rospy_utils'
NNAME = 'test_topic_mapper_node'
from itertools import chain

import roslib
roslib.load_manifest(PKG)
import rospy
import unittest

from std_msgs.msg import (Int32, String)
from rospy_utils import topic_utils as tu


class TestTopicMapperNode(unittest.TestCase):
    def __init__(self, *args):
        super(TestTopicMapperNode, self).__init__(*args)
        rospy.init_node(NNAME)
        # Publishers and Subscribers
        rospy.Subscriber('mapper_output', String, self.callback)
        self.publisher = rospy.Publisher('mapper_input', Int32)

    def setUp(self):
        self.data = None
        self.a_string = tu.get_msg_type('std_msgs/String')('This is a string')
        self.an_integer = tu.get_msg_type('std_msgs/Int32')(42)

    def tearDown(self):
        pass

    def callback(self, msg):
        self.data = msg.data
        self.assertEqual('42', msg.data)

    def test_get_msg_type_returns_correct_msg_type(self):
        self.assertEqual(type(String()), type(self.a_string))
        self.assertEqual('This is a string', self.a_string.data)
        self.assertEqual(type(Int32()), type(self.an_integer))
        self.assertEqual(42, self.an_integer.data)

    def test_topic_mapper(self):
        self.publisher.publish(42)
        self.assertEqual('42', self.data)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_topic_mapper_node',
                   TestTopicMapperNode, sysargs=None)
                   # sysargs=['--cov'])
