#!/usr/bin/env python

#
# author Tom Scherlis

import rospy
import tf
from math import atan2, sqrt

from std_msgs.msg import Header
import numpy as np

from scipy.spatial import transform as stf

class SpotlightAimer:
    def __init__(self):
        self.tfl = tf.TransformListener()
        self.tfb = tf.TransformBroadcaster()

        self.target_frame = "target"
        self.lamp_frame = rospy.get_param("stab_frame")
        self.world_frame = rospy.get_param("world_frame")

        self.output_frame = rospy.get_param("aim_frame")

        self.target_sub = rospy.Subscriber(rospy.get_param("target_topic"), Header, self.update_target_callback)

    # def publish_footprint(self):
    #     # transform the wrench to the base_link:
    #     try:
    #         (trans, rot) = self.tfl.lookupTransform(self.lamp_frame, self.world_frame, rospy.Time(0))
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
    #         print("Failed to find transformation between frames: {}".format(e))
    #         return
    #
    #     # The world frame is located at the base of the lamp, and is axis aligned to the world frame. (No rotation.)
    #     self.tfb.sendTransform(trans, (0, 0, 0, 0), rospy.Time.now(), self.footprint_frame, self.world_frame)

    def publish_target_frame(self, timerEvent):
        try:
            (target_pos, rot) = self.tfl.lookupTransform(self.world_frame, self.target_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Failed to find transformation between frames: {}".format(e))
            return

        self.tfb.sendTransform(target_pos, (0, 0, 0, 1.0), rospy.Time.now(), "lamp_target", self.world_frame)



    def publish_aim_frame(self, timerEvent):
        # Find the vector between the lamp and the target:
        try:
            (lamp_pos, rot) = self.tfl.lookupTransform(self.world_frame, self.lamp_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Failed to find transformation between frames: {}".format(e))
            return

        try:
            (target_pos, rot) = self.tfl.lookupTransform(self.world_frame, self.target_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Failed to find transformation between frames: {}".format(e))
            return

        vector = np.array(target_pos) - np.array(lamp_pos)

        # yaw is rotation about Z
        yaw = atan2(vector[1], vector[0])

        # Pitch is rotation about Y
        pitch = atan2(sqrt(vector[0]**2 + vector[1]**2), vector[2])

        R = stf.Rotation.from_euler("ZYX", [yaw, pitch - np.pi/2, 0])
        quat = tuple(R.as_quat())

        self.tfb.sendTransform(lamp_pos, quat, rospy.Time.now(), self.output_frame, self.world_frame)


    def update_target_callback(self, msg):
        self.target_frame = msg.frame_id;

    def start(self):
        rospy.Timer(rospy.Duration(1.0 / 100), self.publish_aim_frame)
        rospy.Timer(rospy.Duration(1.0 / 100), self.publish_target_frame)
        rospy.spin()

def main():
    rospy.init_node('spotlight_aimer')
    st = SpotlightAimer()
    st.start()
