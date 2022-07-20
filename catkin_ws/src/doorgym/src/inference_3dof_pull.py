#!/usr/bin/env python3

import argparse
import os
import sys
import numpy as np
import torch
import time
import rospy 
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

sys.path.append("../DoorGym")
import a2c_ppo_acktr

from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from arm_operation.srv import * 
from arm_operation.msg import *
from gazebo_msgs.srv import *
from ur5_bringup.srv import *
from gazebo_msgs.srv import *

from scipy.spatial.transform import Rotation as R

from curl_navi import DoorGym_gazebo_utils

class Inference:
    def __init__(self):
        self.joint_state_sub = rospy.Subscriber("/robot/joint_states", JointState, self.joint_state_cb, queue_size = 1)
        self.husky_vel_sub = rospy.Subscriber("/robot/cmd_vel", Twist, self.husky_vel_cb, queue_size=1)
        self.husky_cmd_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)
        self.get_knob_srv = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.goto_pose_srv = rospy.ServiceProxy("/robot/ur5_control_server/ur_control/goto_pose", target_pose)
        self.get_box_pos_srv = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.get_pose_srv = rospy.ServiceProxy("/robot/ur5/get_pose", cur_pose)
        self.ran = rospy.ServiceProxy("husky_ur5/pull_random", Trigger)
        self.arm_go_home = rospy.ServiceProxy("/robot/ur5/go_home", Trigger)
        self.gripper_close = rospy.ServiceProxy("/robot/gripper/close", Trigger)
        self.gripper_open = rospy.ServiceProxy("/robot/gripper/open", Trigger)
        self.joint = np.zeros(23)
        self.dis = 0
        self.listener = tf.TransformListener()
        self.joint_value = joint_value()

        model_path = DoorGym_gazebo_utils.download_model("1_7QLXH7s6VgwktPWLVc0k5gzLFVla70T", "../DoorGym", "husky_ur5_pull_3dof")
        self.actor_critic = DoorGym_gazebo_utils.init_model(model_path, 23)

        self.actor_critic.to("cuda:0")
        self.recurrent_hidden_states = torch.zeros(1, self.actor_critic.recurrent_hidden_state_size)
        
        self.arm_go_home()
        self.ran()
        self.inference()

    def joint_state_cb(self, msg):

        self.joint_value.joint_value[0] = msg.position[7]
        self.joint_value.joint_value[1] = msg.position[6]
        self.joint_value.joint_value[2] = msg.position[5]
        self.joint_value.joint_value[3:] = msg.position[8:]

        self.joint[5:11] = self.joint_value.joint_value

        self.joint[15] = msg.velocity[7]
        self.joint[16] = msg.velocity[6]
        self.joint[17] = msg.velocity[5]
        self.joint[18:21] = msg.velocity[8:]

        self.joint[11:13] = msg.position[2]
        self.joint[21:23] = msg.velocity[2]

        try:
            trans, rot = self.listener.lookupTransform("/base_link", "/odom", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Service call failed: %s"%e)

        _, _, yaw = euler_from_quaternion(rot)

        self.joint[3] = trans[0] - self.dis
        self.joint[4] = yaw
        
        self.dis = trans[0]

    def husky_vel_cb(self, msg):

        self.joint[13] = msg.linear.x
        self.joint[14] = msg.angular.z

    def get_distance(self):

        req = GetLinkStateRequest()
        req.link_name = "pull_box::knob"

        pos = self.get_knob_srv(req)

        try:
            trans, _ = self.listener.lookupTransform("/world", "/object_link", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Service call failed: %s"%e)

        self.joint[0] = trans[0] - pos.link_state.pose.position.x
        self.joint[1] = trans[1] - pos.link_state.pose.position.y
        self.joint[2] = trans[2] - pos.link_state.pose.position.z

    # box
    # def distance(self, x1, y1, x2 = 9.0, y2 = 12.45):
    # cardboard
    def distance(self, x1, y1, x2 = 8.95, y2 = 12.10):

        return ((x1-x2)**2 + (y1-y2)**2)**0.5

    def inference(self):

        begin = time.time()
        closed = False

        while True: 

            target_pose_req = target_poseRequest()
            target_pose_req.factor = 0.8

            res = self.get_pose_srv()

            self.get_distance()
            joint_pose_req = joint_poseRequest()
            joint = torch.from_numpy(self.joint).float().to("cuda:0")
            action, self.recurrent_hidden_states = DoorGym_gazebo_utils.inference(self.actor_critic, joint, self.recurrent_hidden_states)
            next_action = action.cpu().numpy()[0,1,0]

            target_pose_req.target_pose.position.x = res.pose.position.x + 0.001 * next_action[2]
            target_pose_req.target_pose.position.y = res.pose.position.y - 0.0002 * next_action[3]
            target_pose_req.target_pose.position.z = res.pose.position.z + 0.0007 * next_action[4]
            target_pose_req.target_pose.orientation.x = res.pose.orientation.x
            target_pose_req.target_pose.orientation.y = res.pose.orientation.y
            target_pose_req.target_pose.orientation.z = res.pose.orientation.z
            target_pose_req.target_pose.orientation.w = res.pose.orientation.w
   
            # husky
            t = Twist()

            # husky push parameter
            
            t.linear.x = abs(next_action[0]) * 0.023
            t.angular.z = next_action[1] * 0.015

            if(closed):
                t.linear.x *= -1
                t.linear.z *= -1

            res_box = self.get_box_pos_srv("pull_box::link_0","")

            if((self.joint[0] ** 2 + self.joint[1] **2) ** 0.5 <= 0.034 ):
                self.gripper_close()
                rospy.sleep(1)
                if(not closed):
                    target_pose_req.target_pose.position.x = res.pose.position.x 
                    target_pose_req.target_pose.position.y = res.pose.position.y
                    target_pose_req.target_pose.position.z = res.pose.position.z + 0.2
                    self.goto_pose_srv(target_pose_req)
                closed = True
            else:
                self.goto_pose_srv(target_pose_req)
            
            self.husky_cmd_pub.publish(t)
            
            if(self.distance(res_box.link_state.pose.position.x, res_box.link_state.pose.position.y) >= 2.0):
                self.gripper_open()
                self.arm_go_home()
                t.linear.z = -10.0
                self.husky_cmd_pub.publish(t)
                break

        end = time.time()

        print("time", end - begin)

if __name__ == '__main__':
    rospy.init_node("husky_ur5_3dof_pull_node", anonymous=False)
    inference = Inference()
    rospy.spin()