#!/usr/bin/env python

import rospy
from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsResult, TtsFeedback
from pal_detection_msgs.msg import FaceDetections, FaceDetection
from control_msgs.msg import PointHeadAction, PointHeadGoal, PointHeadResult, PointHeadFeedback

from actionlib import SimpleActionClient


class ChipBehaviour(object):
    def __init__(self):
        rospy.loginfo("Initializing ChipBehaviour")

        self.tts_ac = SimpleActionClient('/tts', TtsAction)
        self.tts_ac.wait_for_server()

        self.point_head_ac = SimpleActionClient('/head_controller/point_head_action',
                                                PointHeadAction)
        self.point_head_ac.wait_for_server(rospy.Duration(10.0))

        self.faces_sub = rospy.Subscriber('/pal_face/faces',
                                          FaceDetections,
                                          self.faces_cb,
                                          queue_size=1)

    def faces_cb(self, msg):
        # msg = FaceDetections()
        if len(msg.faces) > 0:
            rospy.loginfo("Face msg: " + str(msg.faces))
            self.say("Oh, face")
            f = msg.faces[0]
            # f = FaceDetection()
            self.look_at(f.position.x, f.position.y, f.position.z)

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
