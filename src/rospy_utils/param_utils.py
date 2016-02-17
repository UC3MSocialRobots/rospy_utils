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

"""
Some utils to ease the use of loading parameters from ROS.

:author: Victor Gonzalez Pacheco (victor.gonzalez.pacheco at gmail.com)
:date: 2014-04
"""
# import roslib
# roslib.load_manifest('rospy_utils')
import rospy

from collections import namedtuple
# from itertools import imap

from .iter_utils import as_iter

Param = namedtuple('Param', 'name value')
# Param.__doc__ = """A struct defining a pair param_name, param_value."""
MASTER_PARAMS = ('/roslaunch', '/rosdistro', '/rosversion', '/run_id')


class ParamNotFoundError(Exception):

    """Error that occurs when is not possible to laod a param."""
    pass


def __get_parameter(pname):
    param_full_name = rospy.search_param(pname)
    if not param_full_name:
        raise ParamNotFoundError("'{}'".format(pname))
    p = Param(param_full_name, rospy.get_param(param_full_name))
    rospy.logdebug("Param '{}'. Value: '{}'".format(p.name, p.value))
    return p


def get_parameters(parameters):
    """
    Yield the params from Parameter Server that are in attribute 'parameters'

    :input: A list parameters to retrieve from the ParamServer
    :yields: A Param("name value) namedtuple
    :raises: ParamNotFoundError in case a parameter Error
    :raises: ValueError in case parameters is empty
    """
    params = as_iter(parameters)
    for pname in params:
        try:
            yield __get_parameter(pname)
        except ParamNotFoundError as pnf:
            pnf.message = pnf.message \
                + "Error when loading parameter '{}'!".format(pname)
            raise


def get_all_user_params():
    """
    Yield all user parameters loaded in the parameter server.

    It yields all parameters except the ones that are
    already loaded by the ROSMaster.

    Attributes:
    :param ns: The namespace where to look for.
    Yields:
    :return param(param_full_name, param_value): yields A param
    """

    all_param_names = rospy.get_param_names()

    for pn in all_param_names:
        if any(map(pn.__contains__, MASTER_PARAMS)):
            continue        # skip parameters that are loaded by rosmaster
        pvalue = rospy.get_param(pn)
        yield Param(pn, pvalue)


def load_params(params):
    """
    Generator that yields the values of the entered params.

    :raises: ParamNotFoundError in case a parameter Error
    :raises: ValueError in case parameters is empty
    """
    for _, pvalue in get_parameters(params):
        yield pvalue


def __attach_parameter(obj, param, create_new=False):
    """
    Modify obj by attaching to it a parameter.

    :param obj: Object to add the parameters
    :param param: parameter to add from the parameter server
    :type param: Param
    :param create_attribs:  (Defautl: False)
                            Wether to add new attributes to the object
                            in case they do not have (see example)
    """
    pname = param.name.rsplit('/', 1)[-1]  # Get rid of param namespace
    if not hasattr(obj, pname) and not create_new:
        raise AttributeError("'{}' does not have attribute '{}'"
                             .format(obj.__class__, pname))
    setattr(obj, pname, param.value)


def attach_parameters(obj, params, create_new=False):
    """
    Attach a list of parameters to a node.

    :param obj: Object to add the parameters
    :param params: name of the params to add from the parameter server
    :type params: a string or a list of strings
    :param create_attribs:  (Defautl: False)
                            Wether to add new attributes to the object
                            in case they do not have (see example)
    :return: obj with new attributes attached

    Example:

        >>> rospy.set_param('p',  'hi!')
        >>> rospy.set_param('p2', 'bye!')
        >>> my_obj = SomeClass(p='zzz')
        >>> print my_obj.p, my_obj.p2
        'zzz'
        AttributeError: 'SomeClass' object has no attribute 'p2'
        >>> my_obj = attach_parameters(my_obj, ['p', p2', create_new=True)
        >>> print my_obj.p, my_obj.p2
        'hi!', 'bye!
    """
    try:
        params = get_parameters(params)
        for p in params:
            __attach_parameter(obj, p, create_new)
        return obj
    except ValueError:
        return obj
    except Exception:
        raise
