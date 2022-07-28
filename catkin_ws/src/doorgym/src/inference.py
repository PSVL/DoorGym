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
from gazebo_msgs.msg import ContactsState
from curl_navi import DoorGym_gazebo_utils
import yaml

class Inference:
    def __init__(self):

        times = rospy.get_param("~times")
        self.dof = rospy.get_param("~dof")

        # metric
        self.count = 0
        self.total = times
        self.success = 0
        self.coi = 0
        self.cnt = 0
        self.collision_states = False

        self.joint_state_sub = rospy.Subscriber("/robot/joint_states", JointState, self.joint_state_cb, queue_size = 1)
        self.husky_vel_sub = rospy.Subscriber("/robot/cmd_vel", Twist, self.husky_vel_cb, queue_size=1)
        self.husky_cmd_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)
        self.get_knob_srv = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.goto_joint_srv = rospy.ServiceProxy("/robot/ur5_control_server/ur_control/goto_joint_pose", joint_pose)
        self.goto_pose_srv = rospy.ServiceProxy("/robot/ur5_control_server/ur_control/goto_pose", target_pose)
        self.get_door_angle_srv = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
        self.get_pose_srv = rospy.ServiceProxy("/robot/ur5/get_pose", cur_pose)
        self.ran = rospy.ServiceProxy("husky_ur5/random", Trigger)
        self.arm_go_home = rospy.ServiceProxy("/robot/ur5/go_home", Trigger)
        self.sub_collision = rospy.Subscriber("/robot/bumper_states", ContactsState, self.cb_collision, queue_size=1)
        self.get_robot_pos = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.joint = np.zeros(23)
        self.dis = 0
        self.listener = tf.TransformListener()
        self.joint_value = joint_value()
        self.total_state = []
        self.total_action = []
        self.total_tra = []
        
        if(self.dof):
            # 3 dof
            model_path = DoorGym_gazebo_utils.download_model("1DR3lRWLNGRVCFsz0IYwEhwL6ZOMd9L5y", "../DoorGym", "husky_ur5_push_3dof")
        else:
            # 6 joints
            model_path = DoorGym_gazebo_utils.download_model("1scp0n_AkGVTnCq80fenGHUfPdO9cwUJl", "../DoorGym", "husky_ur5_push")

        self.actor_critic = DoorGym_gazebo_utils.init_model(model_path, 23)

        self.actor_critic.to("cuda:0")
        self.recurrent_hidden_states = torch.zeros(1, self.actor_critic.recurrent_hidden_state_size)

        self.my_dir = os.path.abspath(os.path.dirname(__file__))

        self.loop()

    def loop(self):

        while(1):

            # initial
            self.ran()

            if(self.count == self.total):
                # finish all goal

                # calculate metric
                s_r = (self.success/self.total) * 100
                f_r = 100 - s_r
                a_c = self.coi / self.total

                # output result 
                d = {'success_rate':s_r, "fail_rate":f_r, "average_coillision":a_c}
                
                # store state,action and model name to yaml file
                if(self.dof):
                    name = "husky_ur5_push_3dof"
                else:
                    name = "husky_ur5_push"
                    
                dis = {'model': name + ".pt",'state':self.total_state, 'action':self.total_action}

                tra = {'environment' : "room_door", "policy": name + ".pt", "trajectories" : self.total_tra}

                with open(os.path.join(self.my_dir,"../"+ name +"_trajectory.yaml"), "w") as f:

                    yaml.dump(tra, f)

                with open(os.path.join(self.my_dir,"../"+ name +"_info.yaml"), "w") as f:

                    yaml.dump(dis, f)

                with open(os.path.join(self.my_dir,"../"+ name +"_result.yaml"), "w") as f:

                    yaml.dump(d, f)

                rospy.loginfo('End')
                break
            else:
                self.count += 1

                if(self.dof):
                    self.inference_3dof()
                else:
                    self.inference_6joints()


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

    def cb_collision(self, msg):
        cnt = 0
        if self.collision_states == True:
            if msg.states == [] and cnt > 1000:
                self.collision_states = False
            else:
                cnt += 1
        elif msg.states != [] and cnt == 0:
            self.collision_states = True
            self.coi += 2
        else:
            self.collision_states = False
            cnt = 0

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


    def inference_3dof(self):

        begin = time.time()
        
        sta = []
        act = []
        robot = []

        while True: 

            robot_pose = self.get_robot_pos("robot", "")
            r_pose = {"position" : [robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z],
                      "orientation" : [robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z, robot_pose.pose.orientation.w]}
            robot.append(r_pose)

            target_pose_req = target_poseRequest()
            target_pose_req.factor = 0.8

            res = self.get_pose_srv()

            self.get_distance()
            joint_pose_req = joint_poseRequest()
            sta.append(self.joint.tolist())
            joint = torch.from_numpy(self.joint).float().to("cuda:0")
            action, self.recurrent_hidden_states = DoorGym_gazebo_utils.inference(self.actor_critic, joint, self.recurrent_hidden_states)
            next_action = action.cpu().numpy()[0,1,0]
            act.append(next_action.tolist())

            req = GetJointPropertiesRequest()
            req.joint_name = "hinge_door_0::hinge"

            res_door = self.get_door_angle_srv(req)

            target_pose_req.target_pose.position.x = res.pose.position.x + 0.001 * next_action[2]
            target_pose_req.target_pose.position.y = res.pose.position.y + 0.001 * next_action[3]
            target_pose_req.target_pose.position.z = res.pose.position.z - 0.001 * next_action[4]
            target_pose_req.target_pose.orientation.x = res.pose.orientation.x
            target_pose_req.target_pose.orientation.y = res.pose.orientation.y
            target_pose_req.target_pose.orientation.z = res.pose.orientation.z
            target_pose_req.target_pose.orientation.w = res.pose.orientation.w

            self.goto_pose_srv(target_pose_req)
            
           # husky
            t = Twist()

            # husky push parameter
            t.linear.x = abs(next_action[0]) * 0.06
            t.angular.z = next_action[1] * 0.015

            self.husky_cmd_pub.publish(t)

            req = GetJointPropertiesRequest()
            req.joint_name = "hinge_door_0::hinge"

            res = self.get_door_angle_srv(req)
            
            if(res.position[0] <= -1.05):
                self.success += 1
                self.total_state.append(sta)
                self.total_action.append(act)
                self.total_tra.append(robot)
                break

            if(time.time() - begin >= 120):
                self.total_state.append(sta)
                self.total_action.append(act)
                self.total_tra.append(robot)
                break

    def inference_6joints(self):

        begin = time.time()

        sta = []
        act = []
        robot = []

        while True: 

            robot_pose = self.get_robot_pos("robot", "")
            r_pose = {"position" : [robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z],
                      "orientation" : [robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z, robot_pose.pose.orientation.w]}
            robot.append(r_pose)

            self.get_distance()
            joint_pose_req = joint_poseRequest()
            sta.append(self.joint.tolist())
            joint = torch.from_numpy(self.joint).float().to("cuda:0")
            action, self.recurrent_hidden_states = DoorGym_gazebo_utils.inference(self.actor_critic, joint, self.recurrent_hidden_states)
            next_action = action.cpu().numpy()[0,1,0]
            act.append(next_action.tolist())
            gripper_action = np.array([next_action[-1], -next_action[-1]])
            joint_action = np.concatenate((next_action, gripper_action))

            req = GetJointPropertiesRequest()
            req.joint_name = "hinge_door_0::hinge"

            res = self.get_door_angle_srv(req)

            # husky ur5 push paramter
            self.joint_value.joint_value[0] += joint_action[2] * 0.004
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
            t.linear.x = abs(joint_action[0]) * 0.05
            t.angular.z = joint_action[1] * 0.002

            self.husky_cmd_pub.publish(t)
            
            if(res.position[0] <= -1.05):
                self.success += 1
                self.total_state.append(sta)
                self.total_action.append(act)
                self.total_tra.append(robot)
                break

            if(time.time() - begin >= 120):
                self.total_state.append(sta)
                self.total_action.append(act)
                self.total_tra.append(robot)
                break

if __name__ == '__main__':
    rospy.init_node("husky_ur5_push_node", anonymous=False)
    inference = Inference()
    rospy.spin()