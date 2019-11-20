#!/usr/bin/env python

#
# author Tom Scherlis

import rospy
import tf
from math import atan2, sqrt

from std_msgs.msg import String
from std_msgs.msg import Float64, Int32
import numpy as np

from scipy.spatial import transform as stf
from math import floor
import random


def az2winds(az):
    az = float(az)
    if az > 0:
        az += np.pi
        return int(az / (2 * np.pi))
    else:
        az -= np.pi
        return int(az / (2 * np.pi))


class SpotlightAimer:
    def __init__(self):
        self.tfl = tf.TransformListener()
        self.tfb = tf.TransformBroadcaster()

        self.target_frame_changed = False
        self.target_frame = ""
        self.lamp_frame = rospy.get_param("lamp_frames/stab_frame")
        self.world_frame = rospy.get_param("lamp_frames/world_frame")

        self.output_frame = rospy.get_param("lamp_frames/aim_frame")

        self.target_sub = rospy.Subscriber(rospy.get_param("target_topic"), String, self.update_target_callback)

        self.lamp_sub = rospy.Subscriber(rospy.get_param("azimuth_controller/topic_from_plant"), Float64,
                                         self.get_lamp_state_callback)
        self.lamp_winds_sub = rospy.Subscriber(rospy.get_param("winds_topic"), Int32, self.get_lamp_winds_callback)

        self.windup_params = rospy.get_param("wrap_probabilities")

        self.lamp_azimuth_local = None
        self.lamp_azimuth_total = None
        self.lamp_winds = None

        self.target_azimuth_total = 0
        self.target_azimuth_local = 0
        self.target_winds = 0

        self.pub_azimuth = rospy.Publisher(rospy.get_param("azimuth_target_topic"), Float64, queue_size=10)
        self.pub_alt = rospy.Publisher(rospy.get_param("alt_target_topic"), Float64, queue_size=10)

    def publish_target_frame(self, timerEvent):
        if self.target_frame == "":
            return
        try:
            (target_pos, rot) = self.tfl.lookupTransform(self.world_frame, self.target_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # print("Failed to find transformation between frames: {}".format(e))
            return

        self.tfb.sendTransform(target_pos, (0, 0, 0, 1.0), rospy.Time.now(), "lamp_target", self.world_frame)

    def aim_update(self, timerEvent):
        if self.target_frame == "":
            return

        # Find the vector between the lamp and the target:
        try:
            (lamp_pos, rot) = self.tfl.lookupTransform(self.world_frame, self.lamp_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # print("Failed to find transformation between frames: {}".format(e))
            return

        try:
            (target_pos, rot) = self.tfl.lookupTransform(self.world_frame, self.target_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # print("Failed to find transformation between frames: {}".format(e))
            return

        vector = np.array(target_pos) - np.array(lamp_pos)

        # yaw is rotation about Z
        new_target_azimuth_local = atan2(vector[1], vector[0])

        # Pitch is rotation about Y
        pitch = atan2(sqrt(vector[0] ** 2 + vector[1] ** 2), vector[2]) - np.pi / 2

        R = stf.Rotation.from_euler("ZYX", [new_target_azimuth_local, pitch, 0])
        quat = tuple(R.as_quat())

        self.tfb.sendTransform(lamp_pos, quat, rospy.Time.now(), self.output_frame, self.world_frame)

        # Determine the absolute azimuth of the target:

        if self.lamp_azimuth_local is None or self.lamp_winds is None:
            return

        if self.target_frame_changed:
            self.target_azimuth_local = new_target_azimuth_local
            # Target frame changed, so we need to calculate a dewinded target:
            self.target_azimuth_total = self.dewind_azimuth_target(self.target_azimuth_local)
            self.target_winds = az2winds(self.target_azimuth_total)
            print("New azimuth: {}, New winds: {} (local azimuth: {})".format(self.target_azimuth_total,
                                                                              self.target_winds,
                                                                              self.target_azimuth_local))
            self.target_frame_changed = False

            self.pub_azimuth.publish(Float64(self.target_azimuth_total))
            self.pub_alt.publish(Float64(-pitch))

        else:
            # if the rotation since the last frame is greater than pi, we probably jumped around the back.
            if abs(new_target_azimuth_local - self.target_azimuth_local) > np.pi:
                if self.target_azimuth_local < 0:
                    print("sub winds!")
                    self.target_winds -= 1
                else:
                    print("add winds!")
                    self.target_winds += 1
            self.target_azimuth_local = new_target_azimuth_local
            # print(self.target_azimuth_local)
            self.target_azimuth_total = self.target_azimuth_local + self.target_winds * 2 * np.pi

        # Publish the alt-azimuth info of the target:
        self.pub_azimuth.publish(Float64(self.target_azimuth_total))
        self.pub_alt.publish(Float64(-pitch))

    # Turn a local target into a global target based on current lamp winds and azimuth.
    # Should be called once per new target.
    def dewind_azimuth_target(self, target_local):
        print("\nDewind target for frame: {}".format(self.target_frame))
        print("Current lamp: {}".format(self.lamp_azimuth_total))
        # first find the travel distance (state to target) in [-pi, pi]
        print("target local: {}, lamp local: {}, diff: {}".format(target_local, self.lamp_azimuth_local,
                                                                  target_local - self.lamp_azimuth_local))
        if target_local - self.lamp_azimuth_local < -np.pi:
            travel = target_local + 2 * np.pi - self.lamp_azimuth_local
            target_abs = self.lamp_azimuth_total + travel

        elif target_local - self.lamp_azimuth_local > np.pi:
            travel = target_local - 2 * np.pi - self.lamp_azimuth_local
            target_abs = self.lamp_azimuth_total + travel

        else:
            travel = target_local - self.lamp_azimuth_local
            target_abs = self.lamp_azimuth_total + travel

        print("target input: {}, travel: {}".format(target_abs, travel))

        # determine if travel will already unwind us:
        if travel * self.lamp_azimuth_total < 0:
            print("result: unwind unnecessary")
            return target_abs

        # find appropriate dewind entry in the dewind table:
        windup_params = {}
        for i in range(len(self.windup_params) - 1, -1, -1):
            if abs(self.lamp_winds) >= self.windup_params[i]["wind_req"]:
                windup_params = self.windup_params[i]
                break
        print("Windup params chosen: {}".format(windup_params["wind_req"]))
        print("soft limit: {}, hard limit: {}, travel: {}".format(windup_params["soft_limit"],
                                                                  windup_params["hard_limit"], travel * 180.0/np.pi))

        # check if we are in an unwind scenario:
        if abs(travel * 180.0 / np.pi) >= windup_params["hard_limit"] \
                or (abs(travel * 180.0 / np.pi) > windup_params["soft_limit"]
                    and random.uniform(0, 100) <= windup_params["probability"]):
            if self.lamp_winds > 0:
                print("result: unwind right (subtract 2pi)")
                # we've winded left, so move the target 2pi to the right to get it to unwind:
                return target_abs - 2 * np.pi
            else:
                print("result: unwind left (add 2pi)")
                # we've winded right, so move the target 2pi to the left to get it to unwind:
                return target_abs + 2 * np.pi

        print("result: unwind condition not met")
        return target_abs

    def update_target_callback(self, msg):
        self.target_frame_changed = self.target_frame != msg.data
        self.target_frame = msg.data

    def get_lamp_winds_callback(self, msg):
        self.lamp_winds = msg.data

    def get_lamp_state_callback(self, msg):
        self.lamp_azimuth_local = (msg.data + np.pi) % (2 * np.pi) - np.pi
        self.lamp_azimuth_total = msg.data

    def start(self):
        rospy.Timer(rospy.Duration(1.0 / 100), self.aim_update)
        rospy.Timer(rospy.Duration(1.0 / 100), self.publish_target_frame)
        rospy.spin()


def main():
    rospy.init_node('spotlight_aimer')
    st = SpotlightAimer()
    st.start()
