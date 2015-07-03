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

"""Utilities to ease the subscription and publication of ros topics."""

# import roslib
# roslib.load_manifest('monarch_multimodal_fusion')
# import argparse
# import rospy

import rospy_utils as rpyu
from rospy_utils import coroutines as co


def get_msg_type(msg_typename):
    """
    Translate from a message typename to a message actual type.

    :param str msg_typename: The typename of the msg.
        Example: 'std_msgs/String'
    :return: The type of the message specified as parameter.
    :rtype: type

    Example:

        >>> msg = get_msg_type("std_msgs/String")
        >>> msg(data='Hello World!')
        data: Hello World!
    """
    msg_package, msg_name = msg_typename.split('/')
    msg_full_typename = '.'.join([msg_package, 'msg', msg_name])
    return rpyu.load_class(msg_full_typename)


class ChainWith(object):

    """Chains two topics with a coroutine."""

    def __init__(self, coroutine, in_topic, in_typeneme,
                 out_topic, out_typename, *args, **kwargs):
        """Constructor."""
        super(ChainWith, self).__init__()
        self.coroutine = coroutine
        self.in_topic = in_topic
        self.in_typeneme = in_typeneme
        self.out_topic = out_topic
        self.out_typename = out_typename

        pipe = co.pipe([coroutine(*args, **kwargs),
                        co.publisher(self.out_topic, self.out_type)])
        co.PipedSubscriber(self.in_topic, self.in_type, pipe)
