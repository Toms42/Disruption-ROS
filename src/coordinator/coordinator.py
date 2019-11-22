import rospy

import numpy as np
from std_msgs.msg import Bool, String, Float64
import random
import tf


class Coordinator:
    def __init__(self):
        self.tfl = tf.TransformListener()

        self.pub_lamp_enable = rospy.Publisher(rospy.get_param("lamp_enable_topic"), Bool, queue_size=10)
        self.pub_target_frame = rospy.Publisher(rospy.get_param("lamp_target_frame_topic"), String, queue_size=10)
        self.pub_bulb_state = rospy.Publisher(rospy.get_param("lamp_on_topic"), Bool, queue_size=10)

        self.min_robot_time = rospy.get_param("time_per_robot_min")
        self.max_robot_time = rospy.get_param("time_per_robot_max")

        self.teleportThreshold = rospy.get_param("teleport_threshold")
        self.hatAlpha = rospy.get_param("hat_alpha")
        self.velocityThreshold = rospy.get_param("velocity_violation_threshold")
        self.velocityPersistence = rospy.get_param("velocity_violation_persistence")

        self.robots = rospy.get_param("robot_frames")
        self.hats = rospy.get_param("hat_frames")
        self.curr_target_frame = None
        self.curr_robot = None

        self.last_swap = None
        self.next_swap = None

        self.lamp_state_sub = rospy.Subscriber(rospy.get_param("lamp_state_topic"), Float64, self.lamp_state_callback)
        self.lamp_target_sub = rospy.Subscriber(rospy.get_param("lamp_target_topic"), Float64,
                                                self.lamp_target_callback)
        self.lamp_on_threshold = rospy.get_param("lamp_on_threshold") * np.pi / 180.0
        self.lamp_state = None
        self.lamp_target = None

        self.hatvelocities = {}
        self.hatpositions = {}
        self.lastViolation = {}

        self.aquired = False
        self.cnt = 0

        self.dt = 1.0 / 100.0

    def update(self, timerEvent):
        # self.pub_lamp_enable.publish(Bool(True))

        lastTarget = self.curr_target_frame

        if self.last_swap is None or rospy.Time.now() > self.next_swap:
            duration = random.uniform(self.min_robot_time, self.max_robot_time)
            self.next_swap = rospy.Time.now() + rospy.Duration(duration)
            self.last_swap = rospy.Time.now()

            self.curr_robot = random.choice([r for r in self.robots if r != self.curr_target_frame])
            print("\nSwitching to robot {}.  Next switch in {} seconds.".format(self.curr_robot, duration))

        # get hat info:
        for hat in self.hats:
            # get tf:
            try:
                (pos, rot) = self.tfl.lookupTransform("world", hat, rospy.Time(0))
                pos = np.array(pos)
                if hat in self.hatpositions:
                    oldposition = self.hatpositions[hat]
                else:
                    oldposition = pos

                self.hatpositions[hat] = pos
                disp = np.linalg.norm(oldposition - pos)
                if disp > self.teleportThreshold:
                    disp = 0

                vel = disp / self.dt
                if hat in self.hatvelocities:
                    oldVel = self.hatvelocities[hat]
                else:
                    oldVel = 0.0

                self.hatvelocities[hat] = (1 - self.hatAlpha) * oldVel + self.hatAlpha * vel

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                # print("Failed to find transformation between frames: {}".format(e))
                pass

        self.curr_target_frame = self.curr_robot
        for hat in self.hatvelocities.keys():
            if self.hatvelocities[hat] > self.velocityThreshold:
                self.lastViolation[hat] = rospy.Time.now()

            if hat in self.lastViolation and \
                    rospy.Time.now() - self.lastViolation[hat] < rospy.Duration(self.velocityPersistence) and \
                    self.curr_target_frame not in self.hats:
                # target the offending hat:
                self.curr_target_frame = hat

                # reset the timer:
                duration = random.uniform(self.min_robot_time, self.max_robot_time)
                self.next_swap = rospy.Time.now() + rospy.Duration(duration)

        if self.curr_target_frame != lastTarget:
            print("Aquisition lost!")
            self.aquired = False
            self.cnt = 0

        if self.lamp_state is not None and self.lamp_target is not None \
                and abs(self.lamp_state - self.lamp_target) < self.lamp_on_threshold:
            if self.cnt < 3:
                self.cnt += 1
            else:
                self.aquired = True

        self.pub_bulb_state.publish(Bool(self.aquired))

        self.pub_target_frame.publish(String(self.curr_target_frame))

    def lamp_state_callback(self, msg):
        self.lamp_state = msg.data

    def lamp_target_callback(self, msg):
        self.lamp_target = msg.data

    def start(self):
        rospy.Timer(rospy.Duration(self.dt), self.update)
        rospy.spin()


def main():
    rospy.init_node('coordinator')
    c = Coordinator()
    c.start()
