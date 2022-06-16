#!/usr/bin/env python3

import os
import sys
import numpy as np
import torch
import time
import rospy 
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

sys.path.append(os.getcwd())
import a2c_ppo_acktr

from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose
from arm_operation.srv import * 
from arm_operation.msg import *
from gazebo_msgs.srv import *
from ur5_bringup.srv import *

class Inference:
    def __init__(self):
        self.husky_cmd_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)
        self.get_knob_srv = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.goto_pose_srv = rospy.ServiceProxy("/robot/ur5_control_server/ur_control/goto_pose", target_pose)
        self.get_door_angle_srv = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
        self.close_srv = rospy.ServiceProxy("/robot/gripper/close", Trigger)
        self.open_srv = rospy.ServiceProxy("/robot/gripper/open", Trigger)
        self.get_pose_srv = rospy.ServiceProxy("/robot/ur5/get_pose", cur_pose)
        self.input = np.zeros(3)
        self.listener = tf.TransformListener()

        load_name = "trained_models/ppo/doorenv-v0_husky-ur5-3dim-load-new.95.pt"
        self.actor_critic, ob_rms = torch.load(load_name)
        self.actor_critic = self.actor_critic.eval()
        self.actor_critic.to("cuda:0")
        self.actor_critic.nn = 3
        self.recurrent_hidden_states = torch.zeros(1, self.actor_critic.recurrent_hidden_state_size)
        self.masks = torch.zeros(1, 1)
        
        req = TriggerRequest()
        self.open_srv(req)
        self.inference()

    def get_knob_xyz(self):

        target_pose_req = target_poseRequest()
        target_pose_req.factor = 0.8

        res = self.get_pose_srv()

        self.input[0] = res.pose.position.x
        self.input[1] = res.pose.position.y
        self.input[2] = res.pose.position.z

    def inference(self):

        begin = time.time()

        while True: 

            self.get_knob_xyz()

            target_pose_req = target_poseRequest()
            target_pose_req.factor = 0.8

            res = self.get_pose_srv()

            output = torch.from_numpy(self.input).float().to("cuda:0")
            with torch.no_grad():
                value, action, _, self.recurrent_hidden_states = self.actor_critic.act(
                    output, self.recurrent_hidden_states, self.masks, deterministic=True)
            next_action = action.cpu().numpy()[0,1,0]
            gripper_action = np.array([next_action[-1], -next_action[-1]])
            print(next_action)

            req = GetJointPropertiesRequest()
            req.joint_name = "hinge_door_0::hinge"

            res_door = self.get_door_angle_srv(req)

            if res_door.position[0] <= -0.2:
                next_action[3] *= -5
                next_action[4] *= -1

            target_pose_req.target_pose.position.x = res.pose.position.x + 0.005 * next_action[2]
            target_pose_req.target_pose.position.y = res.pose.position.y + -0.005 * next_action[3]
            target_pose_req.target_pose.position.z = res.pose.position.z + 0.005 * next_action[4]
            target_pose_req.target_pose.orientation.x = res.pose.orientation.x
            target_pose_req.target_pose.orientation.y = res.pose.orientation.y
            target_pose_req.target_pose.orientation.z = res.pose.orientation.z
            target_pose_req.target_pose.orientation.w = res.pose.orientation.w

            self.goto_pose_srv(target_pose_req)

            # husky
            t = Twist()

            # husky push parameter
            t.linear.x = abs(next_action[0]) * 0.04
            t.angular.z = next_action[1] * 0.015

            self.husky_cmd_pub.publish(t)

            req = GetJointPropertiesRequest()
            req.joint_name = "hinge_door_0::hinge"

            res = self.get_door_angle_srv(req)
            
            if(res.position[0] <= -1.05):
                break

        end = time.time()

        print("time", end - begin)

if __name__ == '__main__':
    rospy.init_node("doorgym_node", anonymous=False)
    inference = Inference()
    rospy.spin()