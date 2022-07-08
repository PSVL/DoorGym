#!/usr/bin/env python3

import rospy
import numpy as np
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
import random

ran = random.uniform(0.0, 1.0)

class init:
    def __init__(self):

        self.pub_info = rospy.Publisher('/state', String, queue_size=10)

        if(ran <= 0.5):
            self.goal = np.array([10.25, 8.52]) 
        else:
            self.goal = np.array([8.0, 8.52])

        self.get_robot_pos = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.reach_state = rospy.Publisher("/reach_goal_state", String, queue_size=1)
        self.reach_state.publish("not_yet")
        rospy.Service("/check_goal", Trigger, self.check_goal)

    def check_goal(self, req):

        res = TriggerResponse()

        if(ran <= 0.5):
            self.pub_info.publish("nav_goal_1")
        else:
            self.pub_info.publish("nav_goal_2")

        robot_pose = self.get_robot_pos("robot","")

        x, y = robot_pose.pose.position.x, robot_pose.pose.position.y
        dis = np.linalg.norm(self.goal - np.array([x, y]))

        if(dis < 0.8):
            self.pub_info.publish("stop")
            self.reach_state.publish("reached")
            res.success = True
        else:
            res.success = False

        return res

if __name__ == '__main__':
    rospy.init_node("check_goal_node", anonymous=False)
    init = init()
    rospy.spin()