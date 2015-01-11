# :version:      0.1
# :copyright:    Copyright (C) 2014 Universidad Carlos III de Madrid.
#                Todos los derechos reservados.
# :licencia      LASR_UC3M v1.0, ver LICENCIA.txt

# Este programa es software libre: puede redistribuirlo y/o modificarlo
# bajo los términos de la Licencia Académica Social Robotics Lab - UC3M
# publicada por la Universidad Carlos III de Madrid, tanto en su versión 1.0
# como en una versión posterior.

# Este programa se distribuye con la intención de que sea útil,
# pero SIN NINGUNA GARANTÍA. Para más detalles, consulte la
# Licencia Académica Social Robotics Lab - UC3M versión 1.0 o posterior.

# Usted ha recibido una copia de la Licencia Académica Social
# Robotics Lab - UC3M en el fichero LICENCIA.txt, que también se encuentra
# disponible en <URL a la LASR_UC3Mv1.0>.

PKG = 'rospy_utils'
import roslib
roslib.load_manifest(PKG)

from rospy_utils.func_utils import (load_class, preconditions,
                                    PreconditionError)
import unittest


class TestFuncUtils(unittest.TestCase):

    """Tests"""

    def __init__(self, *args):
        super(TestFuncUtils, self).__init__(*args)

    def setUp(self):
        pass

    def tearDown(self):
        pass


class TestErrorHandler(unittest.TestCase):
    def __init__(self, *args):
        super(TestErrorHandler, self).__init__(*args)

    def setUp(self):
        pass

    def tearDown(self):
        pass

    @unittest.skip
    def test_reraises(self):
        self.fail()

    @unittest.skip
    def test_uses_logger(self):
        self.fail()

    @unittest.skip
    def test_executes_action(self):
        self.fail()


class TestPreconditions(unittest.TestCase):

    """Tests"""

    def __init__(self, *args):
        super(TestPreconditions, self).__init__(*args)

    def setUp(self):
        self.invalid_precons = ['not_an_int', 1234.567, [], [1, 2, 3]]
        pass

    def tearDown(self):
        pass

    def f1_precons(self, p):
        if type(p) is not int:
            raise PreconditionError("'{}' is not an int".format(p.__class__))

    def __logger(msg):
        print(msg)

    @preconditions(f1_precons, reraise=True)
    def f1(self, p):
        return 2 * p

    @preconditions(f1_precons, reraise=False, logger=__logger, log_msg='f2(): ')
    def f2(self, p):
        return p + p + p

    @preconditions(f1_precons, reraise=False)
    def f3(self, p):
        raise KeyError('f3() raised a "KeyError" error')

    def test_preconditions(self):
        self.assertEqual(4, self.f1(2))

    def test_preconditions_reraises(self):
        for inv_precon in self.invalid_precons:
            with self.assertRaises(PreconditionError):
                self.f1(inv_precon)

    def test_preconditions_not_calls_f_even_if_not_reraises(self):
        # f2 do not reraises the error
        for inv_precon in self.invalid_precons:
            self.assertEqual(None, self.f2(inv_precon))

    def test_preconditions_do_not_catch_errors_in_wrapped_func(self):
        with self.assertRaises(KeyError):
            self.f3(4)

    # @patch.object('TestPreconditions', __logger)
    # def test_preconditions_logger(self, mock_log):
    #     for inv_precon in self.invalid_precons:
    #         mock_log.assert_called_with("'{}' is not an int". \
    #                                     format(inv_precon.__class__))
    #     pass


class DummyClass():

    def __init__(self):
        ''' DummyClass to test the load_class function '''
        self.one = 1
        self.two = 2


class TestLoadClass(unittest.TestCase):

    """Tests for load_class function"""

    def __init__(self, *args):
        super(TestLoadClass, self).__init__(*args)

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_load_class(self):
        dummy = load_class('test_func_utils.DummyClass')
        dummy()


if __name__ == '__main__':
    import rosunit
    #rosunit.unitrun(PKG, 'test_TestFuncUtils', TestFuncUtils)
    rosunit.unitrun(PKG, 'test_FuncUtils_load_class', TestLoadClass)
    rosunit.unitrun(PKG, 'test_FuncUtilsPreconditions', TestPreconditions)
    rosunit.unitrun(PKG, 'test_ErrorHandler', TestErrorHandler)
