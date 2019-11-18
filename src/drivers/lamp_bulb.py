import rospy

from relay import Relay
from std_msgs.msg import Bool
from serial import SerialException


class Bulb:
    def __init__(self):
        print("Starting lamp_bulb driver!")

        self.sub = rospy.Subscriber(rospy.get_param('/driver_params/bulb_topic'), Bool, self.callback)
        self.state = False
        self.ch = rospy.get_param("/driver_params/relay_channel")

        while True:
            try:
                self.relay = Relay(rospy.get_param('/driver_params/relay_tty'))
            except SerialException as se:
                print("Could not configure relay board! Trying again in 3 seconds.")
                print(se.message)
                rospy.sleep(3)
                continue
            break

    def update(self, timerEvent):
        self.relay.setRelay(self.ch, self.state)

    def callback(self, msg):
        self.state = msg.data

    def start(self):
        rospy.Timer(rospy.Duration(1.0 / 50), self.update)
        rospy.spin()


def main():
    rospy.init_node('lamp_bulb')
    b = Bulb()
    b.start()
