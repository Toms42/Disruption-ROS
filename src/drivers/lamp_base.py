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

    def start(self):
        r = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():

            if self.last_cmd is None:
                r.sleep()
                continue
            if rospy.Time.now() - self.last_cmd > rospy.Duration(self.timeout):
                r.sleep()
                continue

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
                    print("Could not configure motor controller after loss!")
                    print(se.message)
            r.sleep()

        # Kill motor if rospy is shutdown:
        self.mc.speed(0)

    def callback(self, msg):
        self.power = msg.data
        self.last_cmd = rospy.Time.now()



def main():
    rospy.init_node('lamp_base')
    lb = LampBase()
    lb.start()
