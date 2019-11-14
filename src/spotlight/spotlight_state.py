import rospy
import numpy as np

import tf
from scipy.spatial import transform as stf

from rcp_spotlight.srv import SetInt32, SetInt32Response

from std_msgs.msg import Float64, Int32


class LampState:
    def __init__(self):
        self.tfl = tf.TransformListener()

        self.winds = 0  # net winds (integer)
        self.azimuth_total = 0  # azimuth total
        self.azimuth_local = 0  # azimuth in -pi to pi range
        self.alt = 0  # altitude

        self.lamp_frame = rospy.get_param("lamp_frames/stab_frame")
        self.world_frame = rospy.get_param("lamp_frames/world_frame")

        self.pub_azimuth = rospy.Publisher(rospy.get_param("azimuth_controller/topic_from_plant"), Float64, queue_size=10)
        self.pub_alt = rospy.Publisher(rospy.get_param("alt_topic"), Float64, queue_size=10)
        self.pub_winds = rospy.Publisher(rospy.get_param("winds_topic"), Int32, queue_size=10)

        self.set_winds_srv = rospy.Service('set_winds', SetInt32, self.set_winds_callback)

    # update the alt/azimuth state based on the measured lamp frame
    def update_state(self, timerEvent):
        # get tf:
        try:
            (pos, rot) = self.tfl.lookupTransform(self.world_frame, self.lamp_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            #print("Failed to find transformation between frames: {}".format(e))
            return

        # get local alt/azimuth angles
        R = stf.Rotation.from_quat(np.array(rot))
        zyx = R.as_euler("ZYX")
        new_azimuth_local = zyx[0]  # [-pi, pi], positive is left.
        self.alt = -zyx[1]  # [-pi, pi]: positive is up for "alt" !!

        # if the rotation since the last frame is greater than pi, we probably jumped around the back.
        if abs(new_azimuth_local - self.azimuth_local) > np.pi:
            if self.azimuth_local < 0:
                self.winds -= 1
            else:
                self.winds += 1

        self.azimuth_local = new_azimuth_local
        self.azimuth_total = self.azimuth_local + 2 * np.pi * self.winds

        self.pub_azimuth.publish(Float64(self.azimuth_total))
        self.pub_alt.publish(Float64(self.alt))
        self.pub_winds.publish(Int32(self.winds))

    def set_winds_callback(self, srv):
        self.winds = srv.data
        res = SetInt32Response()
        res.success = True
        return res

    def start(self):
        rospy.Timer(rospy.Duration(1.0 / 100), self.update_state)
        rospy.spin()


def main():
    rospy.init_node('lamp_state')
    ls = LampState()
    ls.start()
