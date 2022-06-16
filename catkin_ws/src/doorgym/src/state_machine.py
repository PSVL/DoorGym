#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import time
import os
import sys
import numpy as np
import torch
import rospy 
import tf
import random
import tensorflow
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

sys.path.append(os.getcwd())
import a2c_ppo_acktr

from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool, String
from arm_operation.srv import * 
from arm_operation.msg import *
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *

pub_info = rospy.Publisher('/state', String, queue_size=10)

class init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_done'])
        self.arm_home_srv = rospy.ServiceProxy("/robot/ur5/go_home", Trigger)
        self.set_init_pose_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    def execute(self, userdata):
        rospy.loginfo("init position")

        # req = SetModelStateRequest()
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

        req = TriggerRequest()

        self.arm_home_srv(req)

        return 'init_done'

class nav_to_door(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["navigating"])

    def execute(self, userdata):

        pub_info.publish("nav")
        return 'navigating'

class reach_door(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached_door', 'not_yet'])
        self.goal = np.array([9.3, 12.8])
        self.sub_state = rospy.Subscriber("/state_re", String, self.cb_state, queue_size=1)
        self.reach = False

    def cb_state(self, msg):
        
        if(msg.data == "reached"):
            self.reach = True
        else:
            self.reach = False

    def execute(self, userdata):

        if(self.reach):
            return 'reached_door'
        else:
            return 'not yet'

class open_door(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['opening'])

        self.joint_state_sub = rospy.Subscriber("/robot/joint_states", JointState, self.joint_state_cb, queue_size = 1)
        self.husky_vel_sub = rospy.Subscriber("/robot/cmd_vel", Twist, self.husky_vel_cb, queue_size=1)
        self.husky_cmd_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)
        self.get_knob_srv = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.goto_joint_srv = rospy.ServiceProxy("/robot/ur5_control_server/ur_control/goto_joint_pose", joint_pose)
        self.get_door_angle_srv = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
        self.get_odom_sub = rospy.Subscriber("/robot/truth_map_odometry", Odometry, self.get_odom, queue_size=1)
        self.joint = np.zeros(23)
        self.dis = 0
        self.listener = tf.TransformListener()
        self.joint_value = joint_value()

        load_name = "trained_models/ppo/doorenv-v0_push-load.125.pt"
        self.actor_critic, ob_rms = torch.load(load_name)
        self.actor_critic = self.actor_critic.eval()
        self.actor_critic.to("cuda:0")
        self.actor_critic.nn = 23
        self.recurrent_hidden_states = torch.zeros(1, self.actor_critic.recurrent_hidden_state_size)
        self.masks = torch.zeros(1, 1)

    def execute(self, userdata):

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
        req.joint_name = "hinge_door::hinge"

        res = self.get_door_angle_srv(req)

        if res.position[0] <= -0.45:
            joint_action[2] *= -3
            joint_action[6] *= -3
            joint_action[4] *= -0.2

        ## ur5 arm
        self.joint_value.joint_value[0] += joint_action[2] * -0.009
        self.joint_value.joint_value[1] += joint_action[3] * -0.007
        self.joint_value.joint_value[2] += joint_action[4] * -0.007
        self.joint_value.joint_value[3] += joint_action[5] * 0.001
        self.joint_value.joint_value[4] += joint_action[6] * -0.001
        self.joint_value.joint_value[5] += joint_action[7] * 0.001
        
        joint_pose_req.joints.append(self.joint_value)
        res_ = self.goto_joint_srv(joint_pose_req)

        # husky
        t = Twist()

        t.linear.x = abs(joint_action[0]) * 0.04
        t.angular.z = joint_action[1] * 0.015

        self.husky_cmd_pub.publish(t)

        return 'opening'

    def get_odom(self, msg):

        try:
            trans, rot = self.listener.lookupTransform("/base_link", "/map", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Service call failed: %s"%e)

        _, _, yaw = euler_from_quaternion(rot)

        ori = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

        _, _, odom_yaw = euler_from_quaternion(ori)

        self.joint[3] = trans[0] - msg.pose.pose.position.x - self.dis
        self.joint[4] = yaw - odom_yaw

        self.dis = self.joint[3]          

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
        req.link_name = "hinge_door::knob"

        pos = self.get_knob_srv(req)

        try:
            trans, _ = self.listener.lookupTransform("/world", "/object_link", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Service call failed: %s"%e)

        self.joint[0] = trans[0] - pos.link_state.pose.position.x
        self.joint[1] = trans[1] - pos.link_state.pose.position.y
        self.joint[2] = trans[2] - pos.link_state.pose.position.z

class is_open(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_yet', 'opened'])
        self.get_door_angle_srv = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)

    def execute(self, userdata):

        req = GetJointPropertiesRequest()
        req.joint_name = "hinge_door::hinge"

        res = self.get_door_angle_srv(req)

        if(res.position[0] <= -1.05):
            return 'opened'
        else:
            return 'not_yet'

class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigating'])

    def execute(self, userdata):

        pub_info.publish("nav_goal")
        return 'navigating'

class is_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_yet', 'navigated'])
        self.goal = np.array([10.25, 8.52])
        self.get_robot_pos = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    
    def execute(self, userdata):
        
        req = GetModelStateRequest()
        req.model_name = "robot"

        robot_pose = self.get_robot_pos(req)

        x, y = robot_pose.pose.position.x, robot_pose.pose.position.y
        dis = np.linalg.norm(self.goal - np.array([x, y]))

        if(dis < 0.8):
            pub_info.publish("None")
            return 'navigated'
        else:
            pub_info.publish("nav_door")
            return 'not_yet'

def main():

    rospy.init_node("doorgym_node", anonymous=False)

    sm = smach.StateMachine(outcomes=['end'])

    with sm:
        smach.StateMachine.add('init', init(), transitions={'init_done':'nav_to_door'})
        smach.StateMachine.add('nav_to_door', nav_to_door(), transitions={'navigating':'reach_door'})
        smach.StateMachine.add('reach_door', reach_door(), transitions={'reached_door':'open', 'not_yet':'nav_to_door'})
        smach.StateMachine.add('open', open_door(), transitions={'opening':'is_open'})
        smach.StateMachine.add('is_open', is_open(), transitions={'opened':'nav_to_goal', 'not_yet':'open'})
        smach.StateMachine.add('nav_to_goal', Navigation(), transitions={'navigating':'is_goal'})
        smach.StateMachine.add('is_goal', is_goal(), transitions={'not_yet':'nav_to_goal', 'navigated':'end'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()