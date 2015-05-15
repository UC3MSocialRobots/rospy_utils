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
# NNAME = 'test_param_utils'
# import roslib
# roslib.load_manifest(PKG)
# import sys
import rospy
import unittest
from operator import attrgetter

import rospy_utils.param_utils as pu


class DummyClass():
    pass


class ParamUtils_TestCase(unittest.TestCase):

    """Tests the param_utils module"""

    def __init__(self, *args):
        super(ParamUtils_TestCase, self).__init__(*args)

    def setUp(self):
        test1 = pu.Param('/test/test1', 'test_value 1')
        test2 = pu.Param('/test/test2', 'test_value 2')
        test3 = pu.Param('/test/test3', 'test_value 3')
        test4 = pu.Param('/test/more_tests/test4', 'tv 4')
        test5 = pu.Param('/test/more_tests/test5', 'tv 5')
        test6 = pu.Param('/test/more_tests/test6', 'tv 6')

        self.test_params = (test1, test2, test3, test4, test5, test6)
        # Sort by params by name
        self.test_params = sorted(self.test_params, key=attrgetter('name'))
        for p in self.test_params:
            rospy.set_param(p.name, p.value)

    def tearDown(self):
        pass

    # TESTS ####
    def test_get_all_user_parameters(self):
        params = set(pu.get_all_user_params())
        self.assertNotEqual(len(params), 0,
                            "Should have returned a non-empty list")
        self.assertEqual(set(self.test_params) - params, set(),
                         "Not all parameters were retrieved")
        for tp in self.test_params:
            self.assertTrue(tp in params,
                            "Param {} not retrieved from parameter server.")

    def test_get_parameters(self):
        pnames = list(p.name for p in self.test_params)
        params = sorted(pu.get_parameters(pnames), key=attrgetter('name'))
        for i, p in enumerate(params):
            self.assertEqual(p, self.test_params[i],
                             "Retrieved parameters should be equal.\n"
                             "Expected: {}\nRetrieved: {}"
                             .format(self.test_params[i], p))

    def test_get_parameters_raises_TypeError_when_no_attr_passed(self):
        with self.assertRaises(TypeError):
            list(pu.get_parameters())

    def test_get_parameters_raises_ParamNotFoundError_when_not_founds_it(self):
        bad_attrs = {'params': tuple((self.test_params[0].name,
                                      'a/param/that/does/not/exist'))}
        with self.assertRaises(pu.ParamNotFoundError):
            list(pu.get_parameters(bad_attrs))

    def test_load_params(self):
        params = [('p1', 'a'), ('p2', 'ab'), ('p3', 'abc')]
        p1, p2, p3 = [pu.Param(name, value) for name, value in params]
        params = (p1, p2, p3)
        for p in params:
            rospy.set_param(p.name, p.value)
        pnames = list(p.name for p in params)
        rcvd_params = sorted(pu.load_params(pnames), key=len)
        for expected, received in zip(params, rcvd_params):
            self.assertEqual(expected.value, received)

    def test_atach_parameters(self):
        obj = DummyClass()
        pnames, pvalues = zip(*self.test_params)
        obj = pu.attach_parameters(obj, pnames, create_new=True)
        for pname, pvalue in self.test_params:
            self.assertEqual(pvalue, getattr(obj, pname.rsplit('/', 1)[-1]))

    def test_attach_parameters_fails_if_not_allowed_to_create_new_attrib(self):
        obj = DummyClass()
        pnames, pvalues = zip(*self.test_params)
        with self.assertRaises(AttributeError):
            pu.attach_parameters(obj, pnames)

    def test_attach_parameters_returns_same_object_if_no_parameters(self):
        obj = DummyClass()
        self.assertEqual(obj, pu.attach_parameters(obj, []))

    def test_attach_parameters_returns_empty_obj_if_emtpy_obj_is_passed(self):
        self.assertEqual([], pu.attach_parameters([], []))


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_param_utils', ParamUtils_TestCase,
                   # sysargs=['--cov'])
                   sysargs=None)
                       # import rosunit
                       # rosunit.unitrun(PKG, 'test_param_utils',
                       # ParamUtils_TestCase)
