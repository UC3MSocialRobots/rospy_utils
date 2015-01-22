
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

'''
:author: Victor Gonzalez ()
:maintainer: Victor Gonzalez
:version: 0.1

Coroutines that ease the data manipulation and communication between
ROS components.
'''

import rospy
from collections import deque
from functools import wraps

from decorator import decorator


class CoroutineNotConnected(Exception):
    pass


# @decorator
def coroutine(func):
    ''' A decorator function that takes care of starting a coroutine
        automatically on call.

        Example
        -------
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
    '''
    @wraps(func)
    def start(*args, **kwargs):
        cr = func(*args, **kwargs)
        cr.next()
        return cr
    return start


################################################################################
# Filter Coroutines (intermediate steps in the data pipeline)
################################################################################


@coroutine
def buffer(num_items, target=None):
    ''' Accumulates items in a list to send it to target when len == num_items.
        It clears the list after it is sent.

        If you do not specify the target, you will have to send it later.
    '''
    items = list()
    if not target:
        target = (yield)
    while True:
        items.append((yield))
        if len(items) == num_items:
            target.send(items)
            del(items[:])


@coroutine
def sliding_window(size, target=None):
    ''' Sends the last size recived elements to target.

        Example
        -------
        >>> window = sliding_window(3, printer())
        >>> for i in xrange(5):
        >>>     window.send(i)
        [0]
        [0, 1]
        [0, 1, 2]
        [1, 2, 3]
        [2, 3, 4]

    '''
    window = deque([], size)
    if not target:
        target = (yield)
    while True:
        window.append((yield))
        target.send(list(window))


@coroutine
def transformer(f, target=None):
    ''' Applies f to incoming data and sends the result to target coroutine

        Example
        -------
        >>> t = transformer(lambda x: x+1, printer())
        >>> t.send(1)
        2
        >>> t.send(10)
        11
    '''
    if not target:
        target = (yield)
    while True:
        msg = f((yield))
        target.send(msg)


mapper = transformer   # alias


@coroutine
def filter(pred, target=None):
    ''' Coroutine that filters its messages with pred function

        Params
        ------
        :pred: (callable) predicate used to filter incoming messages
        :target: next coroutine in the pipeline

        Example
        -------
        >>> is_even = lambda x: x % 2 == 0
        >>> evens = filter(is_even, printer())
        >>> for i in xrange(5):
        >>>     evens.send(i)
        0
        2
        4

    '''
    if not target:
        target = (yield)
    while True:
        msg = (yield)
        if pred(msg):
            target.send(msg)


@coroutine
def splitter(*coroutines):
    ''' Sends the data to the passed coroutines

        Example
        -------
        >>> s = splitter([printer(),
                          printer(suffix='!!!'),
                          printer(suffix='World')])
        >>> s.send('Hello')
        'Hello'
        'Hello!!!'
        'Hello World!'
    '''
    if not coroutines:
        coroutines = (yield)
    while True:
        data = (yield)
        for c in coroutines:
            c.send(data)


@coroutine
def either(pred, targets=(None, None)):
    ''' Splits an incoming message in two coroutintes according to a predicate.

        Example
        -------


    '''
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
def accumulator(binop, init_value, target=None):
    ''' The reduce equivalent for coroutines:
        Applies binop to each received value and the previous result
        and sends the result to target

        Params:
        -------
        :binop: binary operation to apply to each received message
        :init_value: value used the first time the coroutine receives data
        :target: destination coroutine where to send the accumulated results

        Example:
        -------
        >>> import operator as op
        >>> mul = accumulator(op.mul, 1, printer())
        >>> for _ in xrange(5):
                mul.send(2)
        2
        4
        8
        16
        32
    '''
    value = init_value
    # target = kwargs.get('target', (yield))
    target = target or (yield)
    # if not target:
    #     target = (yield)
    while True:
        value = binop(value, (yield))
        target.send(value)


@coroutine
def dropwhile(pred, target=None):
    ''' Drops received elements until pred is True.

        Example:
        --------
        >>> tw = co.dropwhile(lambda x: 0 <= x <= 1, printer())
        >>> tw.send(-1)     # Nothing is printed
        >>> tw.send(2)      # Nothing is printed
        >>> tw.send(0.2)
        0.2
        >>> tw.send(42)
        42
    '''
    if not target:
        target = (yield)
    value = None
    while not pred(value):
        value = (yield)
    target.send(value)
    while True:
        target.send((yield))


@coroutine
def takewhile(pred, target=None):
    ''' Sends elements to target until pred returns False

        Example:
        --------
        >>> tw = co.takewhile(lambda x: 0 <= x <= 1, printer())
        >>> tw.send(0.1)
        0.1
        >>> tw.send(0.5)
        0.5
        >>> tw.send(2)      # Nothing is printed anymore
        >>> tw.send(0.2)    # Nothing is printed
    '''
    if not target:
        target = ((yield))
    while True:
        val = (yield)
        if pred(val):
            target.send(val)
        else:
            break
    while True:
        _ = (yield)

################################################################################
# Consumer Coroutines (sinks in the data pipeline)
################################################################################


@coroutine
def publisher(topic, msg_type):
    ''' A coroutine-based rospy.publisher

        Params
        ------
        :topic: (str) Name of the topic to publish
        :msg_type: type of the message to publish

        Example of use:
        ---------------

        >>> from std_msgs import String
        >>> pub = publisher('/my_topic', String)
        >>> pub.send("Hello World!")
        >>> # At this point you would receive "Hello World!" in /my_topic

        See: rospy.publisher
    '''
    pub = rospy.Publisher(topic, msg_type)
    while True:
        pub.publish((yield))


@coroutine
def logger(logger, prefix='', suffix=''):
    ''' Calls logger on incoming data

        Example
        -------
        >>> err_logger = logger(rospy.logerr, prefix="ERROR: ", suffix="!!!")
        >>> err_logger.send("This is an error message")
        "ERROR: This is an error message!!!"
    '''
    while True:
        string = str((yield))
        logger(''.join([prefix, string, suffix]))


@coroutine
def printer(prefix='', suffix=''):
    ''' Prints incoming data.

        Example
        -------
        >>> p = printer()
        >>> p.send("Hello World")
        'Hello World'
        >>> p = printer(prefix='You said: ', suffix='!')
        >>> p.send("Hello World")
        'You said: Hello World"'
    '''
    while True:
        item = (yield)
        print(item)


################################################################################
## Utilities
################################################################################
def pipe(coroutines):
    ''' Chains several coroutines together. Returns the first coroutine
        so you can send messages through the whole pipe.

        Note: The pipe establishes the connections between coroutines,
        therefore, you do not need to establish the targets.

        Example
        -------
        >>> coroutines = (transformer(lambda x: x+1),
                          filter(lambda x: x%2==0),
                          printer())
        >>> p = pipe(coroutines)
        >>> p.send(1)
        2
        >>> p.send(4)    # No output

        >>> p.send(-1)
        0

        Example
        -------
        coroutines = [co.sliding_window(3),
                      co.transformer(np.mean),
                      co.filter(lambda x: 0<= x <= 1),
                      co.printer(prefix="Result: ")]

        >>> pipe = co.pipe(coroutines)

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
    '''
    cors = list(reversed(coroutines))
    pairs = zip(cors[:], cors[1:])
    for p in pairs:
        p[1].send(p[0])
    return coroutines[0]
