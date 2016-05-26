#!/usr/bin/env python

import rospy
from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsResult, TtsFeedback
from pal_detection_msgs.msg import FaceDetections, FaceDetection
from actionlib import SimpleActionClient


class ChipBehaviour(object):
    def __init__(self):
        rospy.loginfo("Initializing ChipBehaviour")

        self.ac = SimpleActionClient('/tts', TtsAction)
        self.ac.wait_for_server()

        self.faces_sub = rospy.Subscriber('/pal_face/faces',
                                          FaceDetections,
                                          self.faces_cb,
                                          queue_size=1)

    def faces_cb(self, msg):
        # msg = FaceDetections()
        if len(msg.faces) > 0:
            rospy.loginfo("Face msg: " + str(msg.faces))
            self.say("Oh, face")

    def say(self, text):
        rospy.loginfo("Creating goal with text: " + text)
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = 'en_GB'

        rospy.loginfo("Sending goal!")
        self.ac.send_goal_and_wait(goal)
        rospy.loginfo("We are done!")


if __name__ == '__main__':
    rospy.init_node('robot_talk', anonymous=True)
    cb = ChipBehaviour()
    #cb.say("Sam is an awesome teacher")
    rospy.spin()
