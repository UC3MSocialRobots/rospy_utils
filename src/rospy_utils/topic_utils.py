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


import rospy

# from rospy_utils.param_utils import(load_params)


class TopicMapper():

    ''' Maps a function to a topic and applies that function to every
        incoming message received in that topic.

        It receives messages in input_topic, transforms them to msg_type
        using translator and publishes them to the im_atom topic

        Params:
        -------
        :f: (callable) Transforms in_type msgs out_type
        :in_topic: (str) Name of the topic to subscribe
        :in_type: (type) Type of the messages of the input topic
        :out_topic: (str) output topic name
        :out_type: (type) Type of the messages of the ouput topic

        Kwargs:
        -------
        :logger: ROS logger level (e.g rospy.loginfo, rospy.logdebug, etc.)
                 Defaults to rospy.loginfo
    '''

    def __init__(self, f, in_topic, in_type,
                 out_topic, out_type, **kwargs):
        # name = kwargs.get('node_name', 'mapper')
        self.logger = kwargs.get('logger', rospy.loginfo)

        # rospy.init_node(name, anonymous=True)
        # rospy.on_shutdown(self.shutdown)

        self.logger("Initializing " + rospy.get_name() + " node...")

        self.f = f

        # Subscribers
        rospy.Subscriber(in_topic, in_type, self.callback)
        # Publishers
        self.publisher = rospy.Publisher(out_topic, out_type)

    def callback(self, msg):
        ''' Converts msg to AtomMsg and then publishes it '''
        out_msg = self.f(msg)
        self.logger("Publishing msg: {}".format(str(out_msg)))
        self.publisher.publish(out_msg)

    def run(self):
        ''' Blocks and starts running the node '''
        rospy.spin()

    # def shutdown(self):
    #     ''' Closes the node '''
    #     self.logger('Shutting down ' + rospy.get_name() + ' node.')


# if __name__ == '__main__':
#     try:
        # ''' :todo: pass arguments to node by argv to the file'''
#         rospy.myargv(argv=sys.argv)
#         node = TopicMapper()
#         node.run()
#     except rospy.ROSInterruptException:
#         pass
