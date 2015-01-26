#!/usr/bin/python
'''
:author: Victor Gonzalez Pacheco
:date: 2015-01
'''

PKG = 'rospy_utils'
import unittest
import rospy
from std_msgs.msg import Int32
from rospy_utils import topic_utils as tu


class TestTopicMapper(unittest.TestCase):
    def __init__(self, *args):
        super(TestTopicMapper, self).__init__(*args)
        name = 'test_topic_mapper'
        rospy.init_node(name)

        self.pub = rospy.Publisher('data', Int32)

        self.sub = rospy.Subscriber('transformed', Int32)

    def setUp(self):
        self.inc = lambda x: x + 1
        self.data = range(5)
        self.expected = iter(map(self.inc, self.data))
        self.mapper = tu.TopicMapper(self.inc, 'data', Int32,
                                     'transformed', Int32)

    def cb(self, msg):
        self.assertEqual(msg.data, next(self.expected))

    def tearDown(self):
        pass

    def test_topic_mapper_maps_incoming_messages(self):
        for d in self.data:
            self.pub.publish(d)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_topic_mapper', TestTopicMapper)
