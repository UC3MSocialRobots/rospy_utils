
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
:author: Victor Gonzalez ()
:maintainer: Victor Gonzalez
:version: 0.1

Coroutines that ease the data manipulation and communication in ROS.

"""
from __future__ import print_function

import rospy
from collections import deque
from functools import wraps

# from decorator import decorator


# @decorator
def coroutine(func):
    """
    Decorator function that takes care of starting a coroutine automatically.

    Example

    >>> @coroutine
    >>> def grep(pattern):
    >>> print "Looking for %s" % pattern
    >>> while True:
    >>>     line = (yield)
    >>>     if pattern in line:
    >>>         print line,
    >>> g = grep("python")
    >>> # Notice how you don't need a next() call here
    >>> g.send("Yeah, but no, but yeah, but no")
    >>> g.send("A series of tubes")
    >>> g.send("python generators rock!")

    Unshamely taken from: http://dabeaz.com/coroutines/coroutine.py
    BTW: If you still didn't check out his great tutorials, on coroutines
    Go right now to learn some Python magic tricks, litte bastard:
    http://dabeaz.com/coroutines
    """
    @wraps(func)
    def start(*args, **kwargs):
        """Coroutine wrapper that sets up the coroutine."""
        cr = func(*args, **kwargs)
        next(cr)
        return cr
    return start


###############################################################################
# Filter Coroutines (intermediate steps in the data pipeline)
###############################################################################


@coroutine
def accumulator(num_items, target=None):
    """
    Accumulate items in a list to send it to target when len == num_items.

    It clears the list after it is sent.
    If you do not specify the target, you will have to send it later.


    :param int num_items: (int) Num of items to accumulate before each
    :param target: (default: None) Next coroutine in the data pipeline
        Note that if you don't specify instantiate the coroutine
        specifying the ``target`` you'll have to send it later using
        ``accumulator.send(target)`` method.
    :type target: coroutine or None

    Example

    >>> accum = accumulator(3, printer())
    >>> accum.send(1)     # Nothing sent to printer coroutine
    >>> accum.send(2)     # Still nothing sent
    >>> accum.send(3)
    [1, 2, 3]
    >>> accumf.send(4)
    >>> accumf.send(5)
    >>> accumf.send(6)
    [4, 5, 6]


    Note that you can call ``accumulator`` withouth specifying the target.
    If you don't specify the target, you'll have to send it manually:

    >>> accum = accumulator(3)
    >>> accum.send(printer())     # First you send the target
    >>> accum.send(1)             # And then you can send regular data
    >>> accum.send(2)
    >>> accum.send(3)
    [1, 2, 3]
    """
    items = list()
    if not target:
        target = (yield)
    while True:
        items.append((yield))
        if len(items) == num_items:
            target.send(items)
            del items[:]


@coroutine
def sliding_window(size, target=None):
    """
    Send the last size recived elements to target.

    :param int size: Size of the sliding window
    :param target: (default: None) Next coroutine in the data pipeline
        Note that if you don't specify instantiate the coroutine
        specifying the ``target`` you'll have to send it later using
        ``sliding_window.send(target)`` method.
    :type target: coroutine or None


    Example

    >>> window = sliding_window(3, printer())
    >>> for i in xrange(5):
    >>>     window.send(i)
    [0]
    [0, 1]
    [0, 1, 2]
    [1, 2, 3]
    [2, 3, 4]
    """
    window = deque([], size)
    if not target:
        target = (yield)
    while True:
        window.append((yield))
        target.send(list(window))


@coroutine
def do(f, target=None):
    """
    Apply f on x, send x to the next coroutine.

    Note that ``do`` does not send f(x) but x itself. That means that only
    side effects of ``f`` are relevant.
    This is a good coroutine for logging or publishing messages through
    the pipeline.

    :param callable f: Function with side effects to apply to incoming messages
    :param target: (default: None) Next coroutine in the data pipeline
        Note that if you don't specify instantiate the coroutine
        specifying the ``target`` you'll have to send it later using
        ``do.send(target)`` method.
    :type target: coroutine or None

    Example

    >>> def print_msg(msg):
    >>>     print msg
    >>> t = do(print_msg, transformer(lambda x: x+1, printer()))
    >>> t.send(1)
    1
    2
    >>> t.send(10)
    10
    11
    """
    if not target:
        target = (yield)
    while True:
        msg = (yield)
        f(msg)
        target.send(msg)


@coroutine
def transformer(f, target=None):
    """
    Apply f to incoming data and send the result to target coroutine.

    :param callable f: Function to apply to every incoming message
    :param target: (default: None) Next coroutine in the data pipeline
        Note that if you don't specify instantiate the coroutine
        specifying the ``target`` you'll have to send it later using
        ``transformer.send(target)`` method.
    :type target: coroutine or None

    Example

    >>> t = transformer(lambda x: x+1, printer())
    >>> t.send(1)
    2
    >>> t.send(10)
    11
    """
    if not target:
        target = (yield)
    while True:
        msg = f((yield))
        target.send(msg)


mapper = transformer   # alias


@coroutine
def filterer(pred, target=None):
    """
    Coroutine that Filters its messages with pred function.

    :param callable pred: Predicate that evaluates every incoming message
    :param target: (default: None) Next coroutine in the data pipeline
        Note that if you don't specify instantiate the coroutine
        specifying the ``target`` you'll have to send it later using
        ``filterer.send(target)`` method.
    :type target: coroutine or None


    Example

    >>> is_even = lambda x: x % 2 == 0
    >>> evens = filterer(is_even, printer())
    >>> for i in xrange(5):
    >>>     evens.send(i)
    0
    2
    4

    """
    if not target:
        target = (yield)
    while True:
        msg = (yield)
        if pred(msg):
            target.send(msg)


@coroutine
def splitter(*coroutines):
    """
    Send the data to all the passed coroutines.

    :param coroutines: coroutines at which the incoming data will be sent

    Example:
    >>> s = splitter(printer(),
                     printer(suffix='!!!'),
                     printer(suffix='World!'))
    >>> s.send('Hello')
    'Hello'
    'Hello!!!'
    'Hello World!'

    If you do not specify the coroutines at coroutine instantiation, you'll
    have to do it later by sending a ``tuple`` or a ``list`` using the
    ``send`` method.

    >>> s = splitter()   # You'll need to specify the coroutines later
    >>> targets = [printer(),
                   printer(suffix='!!!'),
                   printer(suffix='World!')]
    >>> s.send(targets)     # Now you can send regular data.
    >>> s.send('Hello')
    'Hello'
    'Hello!!!'
    'Hello World!'
    """
    if not coroutines:
        coroutines = (yield)
    while True:
        data = (yield)
        for c in coroutines:
            c.send(data)


@coroutine
def either(pred, targets=(None, None)):
    """
    Split an incoming message in two coroutintes according to a predicate.

    The predicate is evaluated against incoming data to decide to which
    coroutine resend the incoming message.
    If the predicate produces a ``True``, then the incoming message will be
    sent to ``targets[0]``. If produces ``False`` it will be sent to
    ``targets[1]``

    :param callable pred: Predicate to decide to which target send the data
    :param targets: A pair of coroutines to send the data.
              If you don't instantiate the coroutine with this param,
              you will need to send the targets afterwards prior to start
              sending it data.
    :type targets: tuple(coroutine, coroutine)

    A possible use of this coroutine is to send data to loggers if some
    preconditions fail:

    Example

    >>> data_processor = transformer(lambda x: x**2, printer())
    >>> error_logger = printer(prefix="ERROR: value too high!")
    >>> ei = either(lambda x: x > 10)
    >>> ei.send((data_processor, error_logger))
    >>> ei.send(2)
    4
    >>> ei.send(12)
    "ERROR: value too high!"
    >>> ei.send(5)
    25
    """
    if not all(targets):
        targets = (yield)
    trues, falses = targets
    while True:
        msg = (yield)
        if pred(msg):
            trues.send(msg)
        else:
            falses.send(msg)


@coroutine
def folder(binop, init_value, target=None):
    """
    The reduce equivalent for coroutines.

    Applies binop to each received value and the previous result
    and sends the result to target

    :param callable binop: binary operation to apply to each received msg
    :param init_value: value used the first time the coroutine receives data
    :param target: (default: None) Next coroutine in the data pipeline
        Note that if you don't specify instantiate the coroutine
        specifying the ``target`` you'll have to send it later using
        ``folder.send(target)`` method.
    :type target: coroutine or None

    Example:

    >>> import operator as op
    >>> mul = fold(op.mul, 1, printer())
    >>> for _ in xrange(5):
            mul.send(2)
    2
    4
    8
    16
    32
    """
    value = init_value
    target = target or (yield)
    # target = kwargs.get('target', (yield))
    # if not target:
    #     target = (yield)
    while True:
        value = binop(value, (yield))
        target.send(value)


@coroutine
def dropwhile(pred, target=None):
    """
    Drop received elements while pred is True.

    :param callable pred: Predicate that evaluates incoming data
    :param target: (default: None) Next coroutine in the data pipeline
        Note that if you don't specify instantiate the coroutine
        specifying the ``target`` you'll have to send it later using
        ``buffer.send(target)`` method.
    :type target: coroutine or None

    Example:

    >>> tw = dropwhile(lambda x: 0 <= x <= 1, printer())
    >>> tw.send(-1)     # Nothing is printed
    >>> tw.send(2)      # Nothing is printed
    >>> tw.send(0.2)
    0.2
    >>> tw.send(42)
    42
    """
    if not target:
        target = (yield)
    while True:
        value = (yield)
        if not pred(value):
            target.send(value)
            break
    while True:
        target.send((yield))


@coroutine
def takewhile(pred, target=None):
    """
    Send elements to target until pred returns False.

    :param callable pred: Predicate that evaluates incoming data
    :param target: (default: None) Next coroutine in the data pipeline
        Note that if you don't specify instantiate the coroutine
        specifying the ``target`` you'll have to send it later using
        ``buffer.send(target)`` method.
    :type target: coroutine or None

    Example:
    --------
    >>> tw = takewhile(lambda x: 0 <= x <= 1, printer())
    >>> tw.send(0.1)
    0.1
    >>> tw.send(0.5)
    0.5
    >>> tw.send(2)      # Nothing is printed anymore
    >>> tw.send(0.2)    # Nothing is printed
    """
    if not target:
        target = ((yield))
    while True:
        val = (yield)
        if pred(val):
            target.send(val)
        else:
            break
    while True:
        (yield)

###############################################################################
# Consumer Coroutines (sinks in the data pipeline)
###############################################################################


@coroutine
def publisher(topic, msg_type):
    """
    A coroutine-based rospy.publisher.

    :topic: (str) Name of the topic to publish
    :msg_type: type of the message to publish

    Example of use:

    >>> from std_msgs import String
    >>> pub = publisher('/my_topic', String)
    >>> pub.send("Hello World!")
    >>> # At this point you would receive "Hello World!" in /my_topic

    See: rospy.publisher
    """
    pub = rospy.Publisher(topic, msg_type)
    while True:
        pub.publish((yield))


@coroutine
def logger(logger_, prefix='', suffix=''):
    """
    Call logger_ on incoming data.

    :param callable logger_: Logger function that prints logging messages
    :param str prefix: (Default: '') Prefix to append to incoming data.
    :param str suffix: (Default: '') Suffix to append to incoming data.


    Example:

    >>> err_logger = logger(rospy.logerr, prefix="ERROR: ", suffix="!!!")
    >>> err_logger.send("This is an error message")
    "ERROR: This is an error message!!!"
    """
    while True:
        try:
            string = str((yield))
            logger_(''.join([prefix, string, suffix]))
        except StopIteration:
            pass


@coroutine
def printer(prefix='', suffix=''):
    """
    Print incoming data.

    :param str prefix: (Default: '') Prefix to append to incoming data.
    :param str suffix: (Default: '') Suffix to append to incoming data.

    Example:

    >>> p = printer()
    >>> p.send("Hello World")
    'Hello World'
    >>> p = printer(prefix='You said: ', suffix='!')
    >>> p.send("Hello World")
    'You said: Hello World"'
    """
    while True:
        try:
            item = (yield)
            print(''.join([prefix, item, suffix]))
        except StopIteration:
            pass


###############################################################################
# Data producers
###############################################################################
class PipedSubscriber(object):

    """
    Subscriber to a ROS topic to send/receive msgs to a coroutine or a pipe.

    A wrapper of the `rospy.Subscriber` class that connects the Subscriber
    directly with a coroutine (or a pipe) that processes the incoming msgs.
    In short it has the same api as `rospy.subscriber` but, instead of a
    callback you pass it a coroutine or a `pipe` to it.

    :param str topic_name: Name of the topic to Subscriber
    :param type msg_type: Type of the messages of `topic_name`
    :param generator target: The coroutine or
        ..mod:pipe where the incoming messages will be sent

    Here you can see an example of use:

    >>> ### This node subscribes to the 'my_topic' topic,
    >>> ### transforms incoming strings to uppercase and
    >>> ### publishes them into rospy.logerr
    >>> import rospy
    >>> pipe = pipe([transformer(lambda x: str(x).upper()),
                     logger(rospy.logwarn, prefix='Got: ')])
    >>> rospy.init_node('my_node')
    >>> rospy.loginfo("Node {} started".format(rospy.get_name()))
    >>> PipedSubscriber('my_topic', String, pipe)
    >>> # rospy.spin()
    """

    def __init__(self, topic_name, msg_type, target, *args, **kwargs):
        """ Class constructor. """
        rospy.Subscriber(topic_name, msg_type, target.send, *args, **kwargs)


###############################################################################
# Utilities
###############################################################################
def pipe(coroutines):
    """
    Chain several coroutines to create a data processing pipe.

    Chains several coroutines together and returns the first coroutine
    of the pipe so you can send messages through the whole pipe.

    Note: The pipe establishes the connections between coroutines,
    therefore, you do not need to establish the targets.

    Params
    :param list coroutines: list of coroutines to pipe
    :return: The first coroutine of the pipe
    :rtype: coroutine

    Example

    >>> coroutines = (transformer(lambda x: x+1),
                      filterer(lambda x: x%2==0),
                      printer())
    >>> p = pipe(coroutines)
    >>> p.send(1)
    2
    >>> p.send(4)    # No output
    >>> p.send(-1)
    0


    Here you can see a more useful example where we calculate the mean
    of the last 3 received messages and print them on screen only if they
    meet certain conditions:

    Example

    >>> coroutines = [sliding_window(3),
                      transformer(np.mean),
                      filterer(lambda x: 0<= x <= 1),
                      printer(prefix="Result: ")]
    >>> pipe = pipe(coroutines)
    >>> pipe.send(3)    # No output since mean <= 1
    >>> pipe.send(1)    # No output since mean <= 1
    >>> pipe.send(1)    # No output since mean <= 1
    >>> pipe.send(1)
    1.0
    >>> pipe.send(1)
    1.0
    >>> pipe.send(0.1)
    0.7
    >>> pipe.send(0.1)
    0.4
    """
    _coroutines = list(reversed(coroutines))
    pairs = list(zip(_coroutines[:], _coroutines[1:]))
    for p in pairs:
        p[1].send(p[0])
    return coroutines[0]
