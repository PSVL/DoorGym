#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_msgs.msg import String
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
import random

class init:
    def __init__(self):
        self.arm_home_srv = rospy.ServiceProxy("/robot/ur5/go_home", Trigger)
        self.set_init_pose_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.init_state = rospy.Publisher("/init_state", String, queue_size=1)
        rospy.Service("/go_init", Trigger, self.go_init)

        self.init_state.publish("initing")

    def go_init(self, req):

        res = TriggerResponse()

        req = ModelState()
        req.model_name = 'robot'
        req.pose.position.x = random.uniform(7.0, 11.0)
        req.pose.position.y = 17.0
        req.pose.position.z = 0.1323
        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = -0.707
        req.pose.orientation.w = 0.707

        self.set_init_pose_srv(req)

        self.arm_home_srv()

        res.success = True

        self.init_state.publish("init_finished")

        return res

if __name__ == '__main__':
    rospy.init_node("init_state_node", anonymous=False)
    init = init()
    rospy.spin()