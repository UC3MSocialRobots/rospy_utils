#!/usr/bin/env python

# :copyright:    Copyright (C) 2015 Universidad Carlos III de Madrid.
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

'''This node is a mapper that applies a function to every incoming message
   and sends the output of that function to a topic
                    _________
               x    |       |   f(x)
   in_topic ------> |   f   | -------> out_topic
                    |       |
                    ---------
'''

import roslib
roslib.load_manifest('monarch_multimodal_fusion')
import argparse
import rospy

import rospy_utils as rpyu
from rospy_utils import coroutines as co


def parse_arguments():
    '''Parses all the arguments'''

    parser = argparse.ArgumentParser(
        description="Creates a node that applies a function " +
                    "to every incoming message and sends its output " +
                    "to other topic.")
    parser.add_argument('func',
                        help='Python function to apply to incoming data. ' +
                             'It shuould be a full qualified name. Eg: ' +
                             '"foo.bar.baz" to use "foo.bar.baz()"')
    parser.add_argument("in_topic",
                        help="Topic to subscribe")
    parser.add_argument("in_type",
                        help="Message type of the input topic. "
                             "Example: 'std_msgs/String'")
    parser.add_argument("out_topic",
                        help="Topic to publish")
    parser.add_argument("out_type",
                        help="Message type of the output topic. "
                             "Example: 'std_msgs/String'")

    args = parser.parse_args()
    return args.func, args.in_topic, args.in_type, args.out_topic, args.out_type


def get_msg_type(msg_typename):
    ''' Translates from a message typename to a message actual type

        :param str msg_typename: The typename of the msg.
            Example: 'std_msgs/String'
        :return: The type of the message specified as parameter.
        :rtype: type

        Example:

        >>> msg = get_msg_type("std_msgs/String")
        >>> msg(data='Hello World!')
        data: Hello World!
    '''
    msg_package, msg_name = msg_typename.split('/')
    msg_full_typename = '.'.join([msg_package, 'msg', msg_name])
    return rpyu.load_class(msg_full_typename)

if __name__ == '__main__':
    # TODO: This is highly replicable for filter_node, accumulator, etc.
    #       At some point I will separate all the repeteable parts to a class

    f, in_topic, in_typename, out_topic, out_typename = parse_arguments()
    func = rpyu.load_class(f)
    in_type = get_msg_type(in_typename)
    out_type = get_msg_type(out_typename)

    try:
        rospy.init_node('topic_mapper')
        pipe = co.pipe([co.transformer(func),
                        co.publisher(out_topic, out_type)])
                        # co.logger(rospy.logwarn)])
        subscriber = co.PipedSubscriber(in_topic, in_type, pipe)
        rospy.loginfo("Node {} successfully started\nIt will map: {}\n"
                      "From: {} ({}) --> To: {} ({})"
                      .format(rospy.get_name(),
                              f, in_topic, in_typename,
                              out_topic, out_typename))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
