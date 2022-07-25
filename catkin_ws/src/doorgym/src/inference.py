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

from curl_navi import DoorGym_gazebo_utils

class Inference:
    def __init__(self):
        self.joint_state_sub = rospy.Subscriber("/robot/joint_states", JointState, self.joint_state_cb, queue_size = 1)
        self.husky_vel_sub = rospy.Subscriber("/robot/cmd_vel", Twist, self.husky_vel_cb, queue_size=1)
        self.husky_cmd_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)
        self.get_knob_srv = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.goto_joint_srv = rospy.ServiceProxy("/robot/ur5_control_server/ur_control/goto_joint_pose", joint_pose)
        self.get_door_angle_srv = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
        self.ran = rospy.ServiceProxy("husky_ur5/random", Trigger)
        self.arm_go_home = rospy.ServiceProxy("/robot/ur5/go_home", Trigger)
        self.joint = np.zeros(23)
        self.dis = 0
        self.listener = tf.TransformListener()
        self.joint_value = joint_value()

        model_path = DoorGym_gazebo_utils.download_model("1scp0n_AkGVTnCq80fenGHUfPdO9cwUJl", "../DoorGym", "husky_ur5_push")
        # model_path = DoorGym_gazebo_utils.download_model("1Vy_MAXIizJzv6i3gFNypVEekIyhaEvCH", "../DoorGym", "ur5_push")
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
        req.link_name = "hinge_door_0::knob"

        pos = self.get_knob_srv(req)

        try:
            trans, _ = self.listener.lookupTransform("/world", "/object_link", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Service call failed: %s"%e)

        self.joint[0] = trans[0] - pos.link_state.pose.position.x
        self.joint[1] = trans[1] - pos.link_state.pose.position.y
        self.joint[2] = trans[2] - pos.link_state.pose.position.z

    def inference(self):

        begin = time.time()

        while True: 

            self.get_distance()
            joint_pose_req = joint_poseRequest()
            joint = torch.from_numpy(self.joint).float().to("cuda:0")
            action, self.recurrent_hidden_states = DoorGym_gazebo_utils.inference(self.actor_critic, joint, self.recurrent_hidden_states)
            next_action = action.cpu().numpy()[0,1,0]
            gripper_action = np.array([next_action[-1], -next_action[-1]])
            joint_action = np.concatenate((next_action, gripper_action))

            req = GetJointPropertiesRequest()
            req.joint_name = "hinge_door_0::hinge"

            res = self.get_door_angle_srv(req)

            # husky ur5 push paramter
            self.joint_value.joint_value[0] += joint_action[2] * -0.009
            self.joint_value.joint_value[1] += joint_action[3] * 0.004
            self.joint_value.joint_value[2] += joint_action[4] * 0.004
            self.joint_value.joint_value[3] += joint_action[5] * 0.001
            self.joint_value.joint_value[4] += joint_action[6] * -0.001
            self.joint_value.joint_value[5] += joint_action[7] * 0.001
            
            joint_pose_req.joints.append(self.joint_value)
            res_ = self.goto_joint_srv(joint_pose_req)

            # husky
            t = Twist()

            # husky ur5 push parameter
            t.linear.x = joint_action[0] * 0.03
            t.angular.z = joint_action[1] * 0.015

            self.husky_cmd_pub.publish(t)
            
            if(res.position[0] <= -1.05):
                break

        end = time.time()

        print("time", end - begin)

if __name__ == '__main__':
    rospy.init_node("husky_ur5_node", anonymous=False)
    inference = Inference()
    rospy.spin()