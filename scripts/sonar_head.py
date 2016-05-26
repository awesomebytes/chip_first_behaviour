#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Range
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient


class SonarHead(object):
    def __init__(self):
        rospy.loginfo("Initializing SonarHead")

        self.torso_pub = rospy.Publisher('/torso_controller/command',
                                         JointTrajectory, queue_size=5)
        self.torso_ac = SimpleActionClient('/torso_controller/follow_joint_trajectory',
                                           FollowJointTrajectoryAction)
        self.torso_joints = [0.0, 0.0]
        self.js_sub = rospy.Subscriber('/joint_states',
                                       JointState,
                                       self.js_cb,
                                       queue_size=1)

        # self.torso_ac.wait_for_server()

        self.sonar_sub = rospy.Subscriber('/sonar_torso',
                                          Range,
                                          self.sonar_cb,
                                          queue_size=10)

    def js_cb(self, msg):
        # msg = JointState()
        t1_idx = msg.name.index('torso_1_joint')
        t2_idx = msg.name.index('torso_2_joint')
        self.torso_joints[0] = msg.position[t1_idx]
        self.torso_joints[1] = msg.position[t2_idx]


    def sonar_cb(self, msg):
        # 15 is left 14 is right
        # msg = Range()
        if "14" in msg.header.frame_id:
            # rospy.loginfo("Sonar right distance: " + str(msg.range))
            if msg.range < 0.25:
                #self.move_torso(-0.5, 0.0, time=2.0)
                #self.move_torso_topic(-0.2, 0.0, time=1.0)
                self.move_torso_topic(self.torso_joints[0] - 0.05, 0.0, time=0.2)
        elif "15" in msg.header.frame_id:
            #rospy.loginfo("Sonar left distance: " + str(msg.range))
            if msg.range < 0.25:
                #self.move_torso(0.5, 0.0, time=2.0)
                #self.move_torso_topic(0.2, 0.0, time=1.0)
                self.move_torso_topic(self.torso_joints[0] + 0.05, 0.0, time=0.2)

    def move_torso(self, joint_1, joint_2, time=1.0):
        # -0.24 0.61 torso_2_joint
        # -1.28 1.28 torso_1_joint
        fjtg = FollowJointTrajectoryGoal()
        fjtg.trajectory.joint_names = ['torso_1_joint', 'torso_2_joint']
        p = JointTrajectoryPoint()
        p.positions = [joint_1, joint_2]
        rospy.loginfo("Moving torso to: " + str(p.positions))
        p.time_from_start = rospy.Duration(time)

        fjtg.trajectory.points.append(p)
        self.torso_ac.send_goal_and_wait(fjtg)

    def move_torso_topic(self, joint_1, joint_2, time=1.0):
        jt = JointTrajectory()
        jt.joint_names = ['torso_1_joint', 'torso_2_joint']
        p = JointTrajectoryPoint()
        p.positions = [joint_1, joint_2]
        rospy.loginfo("Moving torso to: " + str(p.positions))
        p.time_from_start = rospy.Duration(time)
        jt.points.append(p)

        self.torso_pub.publish(jt)

if __name__ == '__main__':
    rospy.init_node('demo_sonar', anonymous=True)
    sh = SonarHead()
    # sh.move_torso(-1.0, 0.0, 3.0)
    rospy.spin()
