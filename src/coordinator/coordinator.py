import rospy

from std_msgs.msg import Bool, String
import random

class Coordinator:
    def __init__(self):
        self.pub_lamp_enable = rospy.Publisher(rospy.get_param("lamp_enable_topic"), Bool, queue_size=10)
        self.pub_target_frame = rospy.Publisher(rospy.get_param("lamp_target_frame_topic"), String, queue_size=10)

        self.min_robot_time = rospy.get_param("time_per_robot_min")
        self.max_robot_time = rospy.get_param("time_per_robot_max")

        self.robots = rospy.get_param("robot_frames")
        self.hats = rospy.get_param("hat_frames")
        self.curr_robot = None

        self.last_swap = None
        self.next_swap = None

    def update(self, timerEvent):
        self.pub_lamp_enable.publish(Bool(True))
        # self.pub_target_frame.publish(String("target"))

        if self.last_swap is None or rospy.Time.now() > self.next_swap:

            duration = random.uniform(self.min_robot_time, self.max_robot_time)
            self.next_swap = rospy.Time.now() + rospy.Duration(duration)
            self.last_swap = rospy.Time.now()

            self.curr_robot = random.choice(self.robots)
            pass

    def start(self):
        rospy.Timer(rospy.Duration(1.0 / 100), self.update)
        rospy.spin()

def main():
    rospy.init_node('coordinator')
    c = Coordinator()
    c.start()
