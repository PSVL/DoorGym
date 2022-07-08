#!/usr/bin/env python3

import rospy
import numpy as np
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from geometry_msgs.msg import Twist

class init:
    def __init__(self):
        self.get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.get_angle = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.pub_cmd = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)
        self.sub_goal = np.array([9.0, 13.0])
        rospy.Service("/check_door", Trigger, self.check_door)
        self.pub_info = rospy.Publisher('/state', String, queue_size=10)
        self.reach_state = rospy.Publisher("/reach_door_state", String, queue_size=1)
        self.reach_state.publish("not_yet")

    def check_door(self, req):

        self.pub_info.publish("nav_door")

        res = TriggerResponse()

        agent = self.get_model("robot", "")

        cur = np.array([agent.pose.position.x, agent.pose.position.y])
        dis = np.linalg.norm(self.sub_goal - cur)

        req = GetLinkStateRequest()
        req.link_name = "base_link"
        pos = self.get_angle(req)

        r = R.from_quat([pos.link_state.pose.orientation.x,
                        pos.link_state.pose.orientation.y,
                        pos.link_state.pose.orientation.z,
                        pos.link_state.pose.orientation.w])
        yaw = r.as_euler('zyx')[0]

        if(dis < 0.5):
            self.pub_info.publish("stop")
            rospy.loginfo("reached_door")
            cmd = Twist()
            if(yaw >= -1.45):
                cmd.angular.z = -0.1
            elif(yaw <= -1.65):
                cmd.angular.z = +0.1
            else:
                rospy.loginfo("goal reached")
            self.pub_cmd.publish(cmd)
            self.pub_info.publish("nav_door")

        if(dis < 0.5 and yaw <= -1.45 and yaw >= -1.65):
            self.pub_info.publish("stop")
            self.reach_state.publish("reached")
            res.success = True
        else:
            res.success = False

        return res

if __name__ == '__main__':
    rospy.init_node("check_door_node", anonymous=False)
    init = init()
    rospy.spin()