
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
    @author: Victor Gonzalez Pacheco (victor.gonzalez.pacheco at gmail.com)
    @date: 2014-04

    Some utils to manipulate iterators
'''


def as_iter(obj):
    '''
        Checks if an object is an interable. If not it encapuslates it in a tuple
        @param obj: the object to check if is iterable
        @return: obj if it is iterable, else (obj, )

        Eg.:

            >>> as_iter([1,2,3])
            ... [1,2,3]
            >>> as_iter(123)
            ... (123,)
            >>> as_iter('hello')
            ... ('hello',)
    '''
    return obj if hasattr(obj, '__iter__') else (obj,)
