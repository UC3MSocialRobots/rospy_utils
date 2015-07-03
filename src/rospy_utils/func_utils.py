
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
Utils to ease the use of functions and methods in Python.

:author: Victor Gonzalez Pacheco (victor.gonzalez.pacheco at gmail.com)
:date: 2014-04
"""

from functools import wraps
from contextlib import contextmanager


class PreconditionError(Exception):

    """
    Exception that shuold be raised when the preconditions of a function
    are not met
    """
    pass


@contextmanager
def error_handler(logger=None, log_msg='',
                  action=None, action_args=None, action_kwargs=None,
                  errors=Exception, reraise=False):
    """
    Context Manager that logs errors and executes an action if error occurs.

    :param logger: logging function. Default: None
    :type logger: callable
    :param log_msg: message to add to the logger in case of fail
                    (It will be preced to the exception message)
    :type log_msg: str
    :param action: function to call if an exception occurs. Default: None
    :type action: callable
    :param action_args: argument list to pass to action. Default:[]
    :param action_kwargs: argument keywords to pass to action. Default: {}
    :param errors: exceptions to catch. Default: Exception
    :type errors: Exception or a list of Exceptions
    :param reraise: (default: False) Wether to reraise precondition errors
    """
    try:
        yield
    except errors as e:
        exception_msg = ''.join([log_msg, str(e)])
        if logger:
            logger(str(exception_msg))
        if action:
            action(*action_args, **action_kwargs)
        if reraise:
            raise e


def preconditions(precons, logger=None, log_msg='', errors=PreconditionError,
                  reraise=True):
    """
    Decorator that binds precondition checking to a function.

    :param precons: Preconditions to check
                    Must be a function with same attribs as decorated func
    :type precons: callable
    :param logger: Logger function executed in case of precons fail
                   Default: None
    :param log_msg: Message to add to the logger in case of fail
                    (It will be preced to the exception message)
                    Default: ''
    :type log_msg: str
    :param errors: Exceptions to catch in preconditions.
                   Default: L{PreconditionError}
    :type errors: Exception or a list of Exceptions
    :param reraise: (default: True) Wether to reraise precondition errors

    :return: None if reraise is false and preconditions do not pass
    """
    def decor(f):
        @wraps(f)
        def wrapper(*args, **kwargs):
            with error_handler(logger=logger, log_msg=log_msg,
                               errors=errors, reraise=reraise):
                precons(*args, **kwargs)
                return f(*args, **kwargs)
            return None  # Arrives here only if reraises=False and precons fail
        return wrapper
    return decor


def load_class(full_name):
    """
    Returns an builder of a Python class from its qualified full name.
    To build the object of that class, simply call the returned object.

    Adapted from this SO Answer: http://stackoverflow.com/a/547867/630598

    :type full_name: string
    :param full_name: Class full name: foo.bar.klass
    :return: an instance of foo.bar.Klass. It has to be called just

    Example:

        >>> # Assume Klass is declared in module foo.bar
        >>> class Klass():
        >>>    def __init__(self):
        >>>         print('klass instantiated')

        >>> my_class = load_class('foo.bar.Klass')
        >>> my_class()
        ... 'klass instantiated'
    """
    module_name, klass_name = full_name.rsplit('.', 1)
    mod = __import__(module_name, fromlist=[klass_name])
    klass = getattr(mod, klass_name)
    return klass
