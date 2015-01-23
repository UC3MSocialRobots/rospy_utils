
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
import unittest
from mock import patch
import numpy as np
import operator as op
import rospy
from std_msgs.msg import String

from rospy_utils import coroutines as co

# Utility functions
inc = lambda x: x + 1
is_even = lambda x: x % 2 == 0
is_odd = lambda x: x % 2 != 0


class TestCoroutines(unittest.TestCase):

    def __init__(self, *args):
        super(TestCoroutines, self).__init__(*args)

    @co.coroutine
    def list_evaluator(self, expected):
        ''' Sink coroutine that evaluates received list against expected '''
        while True:
            data = (yield)
            self.assertEqual(expected, data)

    @co.coroutine
    def item_evaluator(self, expected):
        ''' Same as evaluator, but this one expects to receive items 1 by 1 '''
        expected = iter(expected)
        while True:
            data = (yield)
            self.assertEqual(next(expected), data)

    @co.coroutine
    def numeric_evaluator(self, expected):
        ''' Same as item_evaluator, but evaluating with assertaAlmostEqual '''
        expected = iter(expected)
        while True:
            data = (yield)
            self.assertAlmostEqual(next(expected), data)

    def setUp(self):
        self.data = xrange(5)

    def tearDown(self):
        pass

    # Test Buffer ##############################################################
    def __setup_test_buffer(self):
        expected = list(self.data)
        tester = self.list_evaluator(expected)
        return (expected, tester)

    def test_buffer(self):
        expected, tester = self.__setup_test_buffer()
        test_buff = co.buffer(len(self.data), tester)
        for c in self.data:
            # At the end of the loop, co.buffer should send all data
            # to self.evaluator who which will execute the assertion.
            test_buff.send(c)
        # Do it again to ensure the buffer is flushed after sending all the data
        for c in self.data:
            test_buff.send(c)

    def test_buffer_curried(self):
        expected, tester = self.__setup_test_buffer()
        test_buff = co.buffer(len(self.data))
        test_buff.send(tester)
        for c in self.data:
            # At the end of the loop, co.buffer should send all data
            # to self.evaluator who which will execute the assertion.
            test_buff.send(c)

    # Test Sliding Window ######################################################
    def __setup_sliding_window(self):
        expected = [[0], [0, 1], [0, 1, 2], [1, 2, 3], [2, 3, 4]]
        tester = self.item_evaluator(expected)
        return (expected, tester)

    def test_sliding_window(self):
        expected, tester = self.__setup_sliding_window()
        test_sliding = co.sliding_window(3, tester)
        for i in self.data:
            test_sliding.send(i)

    def test_sliding_window_curried(self):
        expected, tester = self.__setup_sliding_window()
        test_sliding = co.sliding_window(3)
        test_sliding.send(tester)
        for i in self.data:
            test_sliding.send(i)

    # Test Transformer #########################################################
    def __set_up_transformer(self):
        expected = map(inc, self.data)
        tester = self.item_evaluator(expected)
        return (expected, tester)

    def test_transformer(self):
        expected, tester = self.__set_up_transformer()
        test_transformer = co.transformer(inc, tester)
        for i in self.data:
            test_transformer.send(i)

    def test_transformer_curried(self):
        expected, tester = self.__set_up_transformer()
        test_transformer = co.transformer(inc)
        test_transformer.send(tester)
        for i in self.data:
            test_transformer.send(i)

    # Test Filter ##############################################################
    def __setup_filter(self):
        expected = filter(is_even, self.data)
        tester = self.item_evaluator(expected)
        return (expected, tester)

    def test_filter(self):
        expected, tester = self.__setup_filter()
        test_filter = co.filter(is_even, tester)
        for i in self.data:
            test_filter.send(i)

    def test_filter_curried(self):
        expected, tester = self.__setup_filter()
        test_filter = co.filter(is_even)
        test_filter.send(tester)
        for i in self.data:
            test_filter.send(i)

    # Test Splitter ############################################################
    def __setup_splitter(self):
        tester1 = self.item_evaluator(self.data)
        tester2 = self.item_evaluator(self.data)
        tester3 = self.item_evaluator(self.data)
        return (tester1, tester2, tester3)

    def test_splitter(self):
        tester1, tester2, tester3 = self.__setup_splitter()
        test_splitter = co.splitter(tester1, tester2, tester3)
        for i in self.data:
            test_splitter.send(i)

    def test_splitter_curried(self):
        testers = self.__setup_splitter()
        test_splitter = co.splitter()
        test_splitter.send(testers)
        for i in self.data:
            test_splitter.send(i)

    # Test Either ##############################################################
    def __setup_either(self):
        evens = filter(is_even, self.data)
        odds = filter(is_odd, self.data)
        tester_evens = self.item_evaluator(evens)
        tester_odds = self.item_evaluator(odds)
        return (tester_evens, tester_odds)

    def test_either(self):
        tester_evens, tester_odds = self.__setup_either()
        test_either = co.either(is_even, (tester_evens, tester_odds))
        for i in self.data:
            test_either.send(i)

    def test_either_curried(self):
        tester_evens, tester_odds = self.__setup_either()
        test_either = co.either(is_even)
        test_either.send((tester_evens, tester_odds))
        for i in self.data:
            test_either.send(i)

    # Test Accumulator #########################################################
    def __setup_accumulator(self):
        expected = [2, 4, 8, 16, 32]
        tester = self.item_evaluator(expected)
        return (expected, tester)

    def test_accumulator(self):
        expected, tester = self.__setup_accumulator()
        test_accumulator_coroutine = co.accumulator(op.mul, 1, tester)
        for _ in xrange(5):
            test_accumulator_coroutine.send(2)

    def test_accumulator_curried(self):
        expected, tester = self.__setup_accumulator()
        test_accumulator_coroutine = co.accumulator(op.mul, 1)
        test_accumulator_coroutine.send(tester)
        for _ in xrange(5):
            test_accumulator_coroutine.send(2)

    # Test Dropwhile ###########################################################
    def __setup_dropwhile(self):
        data = (-1, 2, 0.2, 42, 0.1)
        expected = [0.2, 42, 0.1]
        tester = self.numeric_evaluator(expected)
        return (data, expected, tester)

    def test_dropwhile(self):
        data, expected, tester = self.__setup_dropwhile()
        dropwhile_coroutine = co.dropwhile(lambda x: 0 <= x <= 1, tester)
        for i in data:
            dropwhile_coroutine.send(i)

    def test_dropwhile_curried(self):
        data, expected, tester = self.__setup_dropwhile()
        dropwhile_coroutine = co.dropwhile(lambda x: 0 <= x <= 1)
        dropwhile_coroutine.send(tester)
        for i in data:
            dropwhile_coroutine.send(i)

    # Test Takewhile ###########################################################
    def __setup_takewhile(self):
        data = [0.1, 0.5, 2, 0.2]
        expected = [0.1, 0.5]
        tester = self.numeric_evaluator(expected)
        return (data, expected, tester)

    def test_takewhile(self):
        data, expected, tester = self.__setup_takewhile()
        takewhile_coroutine = co.takewhile(lambda x: 0 <= x <= 1, tester)
        for i in data:
            takewhile_coroutine.send(i)

    def test_takewhile_curried(self):
        data, expected, tester = self.__setup_takewhile()
        takewhile_coroutine = co.takewhile(lambda x: 0 <= x <= 1)
        takewhile_coroutine.send(tester)
        for i in data:
            takewhile_coroutine.send(i)

    # Test Pipe ################################################################
    def test_pipe(self):
        data = (3, 1, 1, 1, 1, 0.1, 0.1, 1000)
        expected = [1.0, 1.0, 0.7, 0.4]
        tester = self.numeric_evaluator(expected)
        coroutines = [co.sliding_window(3),
                      co.transformer(np.mean),
                      co.filter(lambda x: 0 <= x <= 1),
                      tester]
        pipe_tester = co.pipe(coroutines)
        for d in data:
            pipe_tester.send(d)


class TestConsumerCoroutines(unittest.TestCase):

    def __init__(self, *args):
        super(TestConsumerCoroutines, self).__init__(*args)

    def setUp(self):
        pass

    def tearDown(self):
        pass

    @patch('rospy.Publisher')
    def test_publisher(self, mock_pub):
        co.publisher('my_topic', String).send('Hello World!')
        mock_pub.assert_called()

    @patch('rospy.logerr')
    def test_logger(self, mock_loggerr):
        err_logger = co.logger(rospy.logerr, prefix="ERROR: ", suffix="!!!")
        err_logger.send("This is an error message")
        mock_loggerr.assert_called_with("ERROR: This is an error message!!!")

    @patch('__builtin__.print')
    def test_printer(self, mock_print):
        p = co.printer()
        p.send("Hello World!")
        mock_print.assert_called()

    # @patch('__builtin__.print')
    # def test_printer_with_params(self, mock_print):
    #     p = co.printer(prefix='You said: ', suffix='!!!')
    #     p.send("Hello World")
    #     mock_print.assert_called_with('You said: Hello World!!!')

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_coroutines', TestCoroutines)
    rosunit.unitrun(PKG, 'test_consumer_coroutines', TestConsumerCoroutines)
