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
from vx300s_bringup.srv import *
from gazebo_msgs.srv import *

class Inference:
    def __init__(self):
        self.joint_state_sub = rospy.Subscriber("/robot/joint_states", JointState, self.joint_state_cb, queue_size = 1)
        self.husky_vel_sub = rospy.Subscriber("/robot/cmd_vel", Twist, self.husky_vel_cb, queue_size=1)
        self.husky_cmd_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)
        self.get_knob_srv = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.go_joint_srv = rospy.ServiceProxy("/robot/go_joint_pose", joint_pose)
        self.get_door_angle_srv = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
        self.ran = rospy.ServiceProxy("husky_ur5/random", Trigger)
        self.arm_go_home = rospy.ServiceProxy("/robot/ur5/go_home", Trigger)
        self.joint = np.zeros(23)
        self.dis = 0
        self.listener = tf.TransformListener()
        self.joint_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

        load_name = "../DoorGym/model/husky_ur5_push.pt"
        self.actor_critic, ob_rms = torch.load(load_name)
        self.actor_critic = self.actor_critic.eval()
        self.actor_critic.to("cuda:0")
        self.actor_critic.nn = 23
        self.recurrent_hidden_states = torch.zeros(1, self.actor_critic.recurrent_hidden_state_size)
        self.masks = torch.zeros(1, 1)
        
        self.arm_go_home()
        self.ran()
        self.inference()

    def joint_state_cb(self, msg):

        self.joint[5] = msg.position[10]
        self.joint[6] = msg.position[9]
        self.joint[7] = msg.position[0]
        self.joint[8] = msg.position[1]
        self.joint[9] = msg.position[11]
        self.joint[10] = msg.position[12]
        self.joint[11] = msg.position[4]
        self.joint[12] = msg.position[4]

        self.joint[15] = msg.velocity[10]
        self.joint[16] = msg.velocity[9]
        self.joint[17] = msg.velocity[0]
        self.joint[18] = msg.velocity[1]
        self.joint[19] = msg.velocity[11]
        self.joint[20] = msg.velocity[12]
        self.joint[21] = msg.velocity[4]
        self.joint[22] = msg.velocity[4]

        self.joint_value = self.joint[5:11]

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
            trans, _ = self.listener.lookupTransform("/world", "vx300s/ee_gripper_link", rospy.Time(0))
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
            with torch.no_grad():
                value, action, _, self.recurrent_hidden_states = self.actor_critic.act(
                    joint, self.recurrent_hidden_states, self.masks, deterministic=True)
            next_action = action.cpu().numpy()[0,1,0]
            gripper_action = np.array([next_action[-1], -next_action[-1]])
            joint_action = np.concatenate((next_action, gripper_action))

            req = GetJointPropertiesRequest()
            req.joint_name = "hinge_door_0::hinge"

            res = self.get_door_angle_srv(req)

            # vx300s push paramter
            self.joint_value[0] += joint_action[2] * -0.009
            self.joint_value[1] += joint_action[3] * 0.004
            self.joint_value[2] += joint_action[4] * 0.004
            self.joint_value[3] += joint_action[5] * 0.001
            self.joint_value[4] += joint_action[6] * -0.001
            self.joint_value[5] += joint_action[7] * 0.001

            req_joint = joint_poseRequest()
            req_joint.joint_value = list(self.joint_value)
            res_ = self.go_joint_srv(req_joint)

            # husky
            t = Twist()

            # husky push parameter
            t.linear.x = abs(joint_action[0]) * 0.03
            t.angular.z = joint_action[1] * 0.015

            self.husky_cmd_pub.publish(t)
            
            if(res.position[0] <= -1.05):
                break

        end = time.time()

        print("time", end - begin)

if __name__ == '__main__':
    rospy.init_node("husky_vx300s_node", anonymous=False)
    inference = Inference()
    rospy.spin()