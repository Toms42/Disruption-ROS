from smc import SMC

from serial import SerialException

import rospy
from std_msgs.msg import Float64


class LampBase:
    def __init__(self):
        print("Starting lamp_base driver!")

        self.sub = rospy.Subscriber("base_motor_power", Float64, self.callback)

        mc_tty = rospy.get_param('/driver_params/smc_tty')
        self.minspeed = rospy.get_param('/driver_params/smc_speed_min')
        self.maxspeed = rospy.get_param('/driver_params/smc_speed_max')

        self.power = 0

        print("Connecting to motor controller at {}".format(mc_tty))

        while True:
            try:
                self.mc = SMC(mc_tty, 115200)
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
            speedcmd = round((self.maxspeed - self.minspeed) / 2 * self.power)
            #print(self.power)
            #print(speedcmd)
            self.mc.speed(int(speedcmd))
            r.sleep()

    def callback(self, msg):
        self.power = msg.data


def main():
    print("here!")
    rospy.init_node('lamp_base')
    lb = LampBase()
    lb.start()
