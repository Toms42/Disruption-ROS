import rospy

from std_msgs.msg import Bool, String

class Coordinator:
    def __init__(self):
        self.pub_lamp_enable = rospy.Publisher(rospy.get_param("lamp_enable_topic"), Bool, queue_size=10)
        self.pub_target_frame = rospy.Publisher(rospy.get_param("lamp_target_frame_topic"), String, queue_size=10)

    def update(self, timerEvent):
        self.pub_lamp_enable.publish(Bool(True))
        self.pub_target_frame.publish(String("target"))

    def start(self):
        rospy.Timer(rospy.Duration(1.0 / 100), self.update)
        rospy.spin()

def main():
    rospy.init_node('coordinator')
    c = Coordinator()
    c.start()
