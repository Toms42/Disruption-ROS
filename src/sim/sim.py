import rospy
import numpy as np
from std_msgs.msg import Float64

from scipy.spatial import transform as stf
import tf

class Sim:
    # simulates the plant using a very simple velocity model
    def __init__(self):
        self.tfb = tf.TransformBroadcaster()

        self.sub_effort = rospy.Subscriber("/drivers/azimuth_effort", Float64, self.callback)
        self.frame = "lamp_stab"
        self.omega_max = 2 * 2 * np.pi  # 2 rotations per second
        self.azimuth = 0
        self.position = [0, 0, 0]
        self.effort = 0
        self.dt = 0.01

    def update(self, timerEvent):
        self.azimuth += self.effort * self.omega_max * self.dt

        R = stf.Rotation.from_euler("ZYX", [self.azimuth, 0, 0])
        quat = tuple(R.as_quat())

        self.tfb.sendTransform(self.position, quat, rospy.Time.now(), self.frame, "world")

    def start(self):
        rospy.Timer(rospy.Duration(self.dt), self.update)
        rospy.spin()

    def callback(self, msg):
        self.effort = msg.data


def main():
    rospy.init_node('sim')
    s = Sim()
    s.start()
