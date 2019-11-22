import time
import random
import rospy
from std_msgs.msg import String, Bool

class RobotNode:
	def __init__(self):
		self.enable = False
		self.target = ""

		self.target_sub = rospy.Subscriber(rospy.get_param("/spotlight/target_topic"), String, self.update_target)
		self.enable_sub = rospy.Subscriber(rospy.get_param("/spotlight/enable_topic"), Bool, self.update_enable)
		self.file = rospy.get_param("~file")

	def update_robots(self, timerEvent):
		if self.enable:
			enable = "True"
		else:
			enable = "False"

		enable = "True"

		with open(self.file, 'w+') as f:
			f.write("{}:{}:\n".format(self.target, enable))
			#time.sleep(10000)

		#print("{}:{}:\n".format(self.target, enable))

	def update_target(self, msg):
		self.target = msg.data

	def update_enable(self, msg):
		self.enable = msg.data

	def start(self):
		rospy.Timer(rospy.Duration(1.0/3.0), self.update_robots)
		rospy.spin()


def main():
	rospy.init_node('robot_node')
	r = RobotNode()
	r.start()
