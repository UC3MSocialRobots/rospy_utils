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
# import rospy

from rospy_utils.iter_utils import as_iter
import unittest


class TestIterUtils(unittest.TestCase):

    """Tests"""

    def __init__(self, *args):
        super(TestIterUtils, self).__init__(*args)

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_as_iter(self):
        self.assertEqual([1, 2, 3], as_iter([1, 2, 3]))
        self.assertEqual((123,), as_iter(123))
        self.assertEqual(('hello',), as_iter('hello'))
        self.assertEqual([], as_iter([]))
        self.assertEqual(list(iter([1, 2, 3])), as_iter(list(iter([1, 2, 3]))))


if __name__ == '__main__':

    import rosunit
    rosunit.unitrun(PKG, 'test_TestIterUtils', TestIterUtils)
