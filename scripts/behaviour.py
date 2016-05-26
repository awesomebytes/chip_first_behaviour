#!/usr/bin/env python
# MoveIt tutorial says we need this:
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import rospy
from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsResult, TtsFeedback
from pal_detection_msgs.msg import FaceDetections, FaceDetection
from control_msgs.msg import PointHeadAction, PointHeadGoal, PointHeadResult, PointHeadFeedback

from actionlib import SimpleActionClient


class ChipBehaviour(object):
    def __init__(self):
        rospy.loginfo("Initializing ChipBehaviour")

        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        self.group = moveit_commander.MoveGroupCommander("right_arm_torso")

        self.pose_pub = rospy.Publisher('/arm_pointing_pose',
                                        geometry_msgs.msg.PoseStamped,
                                        queue_size=1)

        self.tts_ac = SimpleActionClient('/tts', TtsAction)
        self.tts_ac.wait_for_server()

        self.point_head_ac = SimpleActionClient('/head_controller/point_head_action',
                                                PointHeadAction)
        self.point_head_ac.wait_for_server(rospy.Duration(10.0))

        self.faces_sub = rospy.Subscriber('/pal_face/faces',
                                          FaceDetections,
                                          self.faces_cb,
                                          queue_size=1)

    def point_at(self, x, y, z):
        # Get shoulder pose
        shoulder_pose = geometry_msgs.msg.Pose()
        shoulder_pose.position.x = -0.21
        shoulder_pose.position.y = -0.22
        shoulder_pose.position.z = 1.28

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 1.0

        pose_target.position.y = x - 0.4
        max_z = 3.0
        if z > max_z:
            normalized_z = 1.0
        elif z < 0.0:
            normalized_z = 0.0
        else:
            normalized_z = z / 3.0
        normalized_z = normalized_z / 3.0
        pose_target.position.x = normalized_z - 0.05
        pose_target.position.z = 1.3
        rospy.loginfo("Sending arm to: " + str(pose_target))
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = '/odom'
        ps.pose = pose_target
        self.pose_pub.publish(ps)
        self.group.set_pose_target(pose_target)

        plan1 = self.group.plan()
        self.group.go(wait=True)

    def faces_cb(self, msg):
        # msg = FaceDetections()
        if len(msg.faces) > 0:
            #rospy.loginfo("Face msg: " + str(msg.faces))
            self.say("Oh, face")
            f = msg.faces[0]
            # f = FaceDetection()
            # self.look_at(f.position.x, f.position.y, f.position.z)
            self.point_at(f.position.x, f.position.y, f.position.z)

    def look_at(self, x, y, z):
        phg = PointHeadGoal()
        phg.target.header.frame_id = '/stereo_optical_frame'
        phg.target.point.x = x
        phg.target.point.y = y
        phg.target.point.z = z

        phg.pointing_axis.z = 1.0
        phg.pointing_frame = phg.target.header.frame_id
        phg.min_duration = rospy.Duration(1.0)
        phg.max_velocity = 1.0

        self.point_head_ac.send_goal_and_wait(phg)

    def say(self, text):
        rospy.loginfo("Creating goal with text: " + text)
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = 'en_GB'

        rospy.loginfo("Sending goal!")
        self.tts_ac.send_goal_and_wait(goal)
        rospy.loginfo("We are done!")


if __name__ == '__main__':
    rospy.init_node('robot_talk', anonymous=True)
    cb = ChipBehaviour()
    # cb.say("Sam is an awesome teacher")
    # cb.look_at(-2.0, 0.0, 2.0)
    rospy.spin()
