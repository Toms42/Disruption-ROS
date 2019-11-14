from maestro import Maestro

import rospy
from std_msgs.msg import Float64
import numpy as np


class LampBase:
    def __init__(self):
        print("Starting lamp_base driver!")

        self.sub = rospy.Subscriber(rospy.get_param('/driver_params/altitude_topic'), Float64, self.callback)

        maestro_tty = rospy.get_param('/driver_params/maestro_tty')
        self.minAngle = rospy.get_param('/driver_params/servo_min_deg')
        self.maxAngle = rospy.get_param('/driver_params/servo_max_deg')
        self.minCmd = rospy.get_param('/driver_params/servo_min_us')
        self.maxCmd = rospy.get_param('/driver_params/servo_max_us')
        self.channel = rospy.get_param('/driver_params/servo_channel')

        self.angle = None

        print("Connecting to servo controller at {}".format(maestro_tty))

        while True:
            try:
                self.maestro = Maestro(ttyStr=maestro_tty)
            except ValueError as se:
                print("Could not configure motor controller! Trying again in 3 seconds.")
                print(se.message)
                rospy.sleep(3)
                continue
            break

        print("Successfully initialized tilt servo driver!")

    def start(self):
        r = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():

            if self.angle is None:
                r.sleep()
                continue

            cmd = self.minCmd + (self.maxCmd - self.minCmd) * \
                  (self.angle - self.minAngle) / float(self.maxAngle - self.minAngle)
            print(cmd)
            self.maestro.setTarget(int(cmd * 4), self.channel)
            r.sleep()

        # Kill motor if rospy is shutdown:
        self.mc.speed(0)

    def callback(self, msg):
        msg.data = msg.data * 180.0 / np.pi
        if msg.data < self.minAngle:
            msg.data = self.minAngle
        elif msg.data > self.maxAngle:
            msg.data = self.maxAngle
        self.angle = msg.data


def main():
    print("here!")
    rospy.init_node('lamp_base')
    lb = LampBase()
    lb.start()
