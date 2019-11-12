#!/usr/bin/env python

#
# author Tom Scherlis

import rospy
import tf


class SpotlightAimer:
    def __init__(self):
        self.tfl = tf.TransformListener()
        self.target_frame = ""
        self.lamp_frame = rospy.get_param("spotlight/head_frame")
        self.lamp_stabilized_frame = rospy.get_param("spotlight/tabilized_frame")

        self.output_frame = rospy.get_param("spotlight/aim_frame")

    def frameUpdate(self):



def main():
    rospy.init_node('spotlight_aimer')
    st = SpotlightAimer()
    st.start()
