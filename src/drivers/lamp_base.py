from smc import SMC

from serial import SerialException

import rospy
from std_msgs.msg import Float64


class LampBase:
    def __init__(self):
        print("Starting lamp_base driver!")

        self.sub = rospy.Subscriber(rospy.get_param('/driver_params/effort_topic'), Float64, self.callback)

        self.mc_tty = rospy.get_param('/driver_params/smc_tty')
        self.minspeed = rospy.get_param('/driver_params/smc_speed_min')
        self.maxspeed = rospy.get_param('/driver_params/smc_speed_max')
        self.invert = rospy.get_param('/driver_params/smc_invert')

        self.power = 0
        self.last_cmd = None
        self.timeout = 0.2
        self.mc = None

        print("Connecting to motor controller at {}".format(self.mc_tty))

        while True:
            try:
                self.mc = SMC(self.mc_tty, 115200)
                self.mc.init()
            except SerialException as se:
                print("Could not configure motor controller! Trying again in 3 seconds.")
                print(se.message)
                rospy.sleep(3)
                continue
            break

        print("Successfully initialized lamp base motor driver!")

    def timerUpdate(self, timerEvent):
        if self.last_cmd is None:
            return
        if rospy.Time.now() - self.last_cmd > rospy.Duration(self.timeout):
            return

        speedcmd = float(self.maxspeed - self.minspeed) / 2 * self.power
        if self.invert:
            speedcmd = -speedcmd

        try:
            self.mc.speed(int(speedcmd))
        except SerialException:
            try:
                self.mc = SMC(self.mc_tty, 115200)
                self.mc.init()
            except SerialException as se:
                print("Could not configure motor controller after connection  loss!.")
                print(se.message)

    def callback(self, msg):
        self.power = msg.data
        self.last_cmd = rospy.Time.now()

    def start(self):
        rospy.Timer(rospy.Duration(1.0/50), self.timerUpdate)
        rospy.spin()
        self.mc.speed(0)

def main():
    rospy.init_node('lamp_base')
    lb = LampBase()
    lb.start()
