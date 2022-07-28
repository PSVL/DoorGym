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

sys.path.append("../DoorGym")
import a2c_ppo_acktr

from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool, String, Float32
from arm_operation.srv import * 
from arm_operation.msg import *
from ur5_bringup.srv import *
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *

from curl_navi import DoorGym_gazebo_utils
import yaml

pub_info = rospy.Publisher('/state', String, queue_size=10)
goal_pub = rospy.Publisher("/tare/goal", PoseStamped, queue_size=1)

my_dir = os.path.abspath(os.path.dirname(__file__))
# read yaml
with open(os.path.join(my_dir,"../goal.yaml"), 'r') as f:
    data = yaml.load(f)

goal_totoal = data['goal']
goal = []

# metric
count = 0
total = len(goal_totoal)
success = 0
coi = 0

begin = time.time()

class loop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['loop_done', 'looping'])

    def execute(self, userdata):

        global count, total, success, coi

        if(count == total):
            # finish all goal

            # calculate metric
            s_r = (success/total) * 100
            f_r = 100 - s_r
            a_c = coi / total

            # output result 
            d = {'success_rate':s_r, "fail_rate":f_r, "average_coillision":a_c}

            with open(os.path.join(my_dir,"../" + method + ".yaml"), "w") as f:
                yaml.dump(d, f)
            
            rospy.loginfo('End')
            return 'loop_done'
        else:
            goal = goal_totoal[count]
            return 'looping'

class init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_done'])
        self.arm_home_srv = rospy.ServiceProxy("/robot/ur5/go_home", Trigger)
        self.set_init_pose_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.set_door_srv = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

    def execute(self, userdata):

        print("init")
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

        # set robot
        self.set_init_pose_srv(req)

        # set door 
        self.set_door_srv("hinge_door_0", "", ["hinge"], [0])
        
        # arm go home
        self.arm_home_srv()

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
        self.get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.get_angle = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.pub_cmd = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)
        self.sub_goal = np.array([9.0, 13.0])

        # publish goal to tare
        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.pose.position.x = self.sub_goal[0]
        pose.pose.position.y = self.sub_goal[1]

        goal_pub.publish(pose)

    def execute(self, userdata):

        global begin

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
            pub_info.publish("stop")
            rospy.loginfo("reached_door")
            cmd = Twist()
            if(yaw >= -1.45):
                cmd.angular.z = -0.1
            elif(yaw <= -1.65):
                cmd.angular.z = +0.1
            else:
                rospy.loginfo("goal reached")
            self.pub_cmd.publish(cmd)

        if(dis < 0.5 and yaw <= -1.45 and yaw >= -1.65):
            pub_info.publish("stop")
            return 'reached_door'
        elif(time.time() - begin >= 180):
            count += 1
            return 'reached_door'
        else:
            return 'not_yet'

class open_door(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['opening'])

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
        self.cnt = 0
        self.listener = tf.TransformListener()
        self.joint_value = joint_value()
        
        self.collision_states = False

        if(method == "DoorGym"):
            model_path = DoorGym_gazebo_utils.download_model("1Vy_MAXIizJzv6i3gFNypVEekIyhaEvCH", "../DoorGym", "ur5_push")
            self.sub_collision_gym = rospy.Subscriber("/robot/bumper_states", ContactsState, self.cb_collision_gym, queue_size=1)
        elif(method == "RL_mm"):
            model_path = DoorGym_gazebo_utils.download_model("1scp0n_AkGVTnCq80fenGHUfPdO9cwUJl", "../DoorGym", "husky_ur5_push")
            self.sub_collision = rospy.Subscriber("/robot/bumper_states", ContactsState, self.cb_collision, queue_size=1)
        
        self.actor_critic = DoorGym_gazebo_utils.init_model(model_path, 23)

        self.actor_critic.to("cuda:0")
        self.recurrent_hidden_states = torch.zeros(1, self.actor_critic.recurrent_hidden_state_size)

    def execute(self, userdata):

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

        return 'opening'

    def cb_collision(self, msg):

        global coi
        count = 0
        if self.collision_states == True:
            if msg.states == [] and count > 1000:
                self.collision_states = False
            else:
                count += 1
        elif msg.states != [] and count == 0:
            self.collision_states = True
            coi += 1
        else:
            self.collision_states = False
            count = 0

    def cb_collision_gym(self, msg):
        if self.collision_states == True:
            if msg.states == [] and self.cnt > 1000:
                self.collision_states = False
            else:
                self.cnt += 1
        elif msg.states != [] and self.cnt == 0:
            self.collision_states = True
            self.coi += 1
        else:
            self.collision_states = False
            self.cnt = 0

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

    def husky_vel_cb(self, msg):

        self.joint[13] = msg.linear.x
        self.joint[14] = msg.angular.z

    def get_distance(self):

        req = GetLinkStateRequest()
        req.link_name = "hinge_door_0::knob"

        pos = self.get_knob_srv(req)

        try:
            trans, _ = self.listener.lookupTransform("/map", "/object_link", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Service call failed: %s"%e)

        self.joint[0] = trans[0] - pos.link_state.pose.position.x
        self.joint[1] = trans[1] - pos.link_state.pose.position.y
        self.joint[2] = trans[2] - pos.link_state.pose.position.z

class is_open(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_yet', 'opened'])
        self.get_door_angle_srv = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
        self.arm_go_home = rospy.ServiceProxy("/robot/ur5/go_home", Trigger)
        self.pub_cmd = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)

    def execute(self, userdata):

        global begin

        req = GetJointPropertiesRequest()
        req.joint_name = "hinge_door_0::hinge"

        res = self.get_door_angle_srv(req)

        if(res.position[0] <= -1.05):
            cmd = Twist()
            cmd.linear.x = -25.0
            self.pub_cmd.publish(cmd)
            self.arm_go_home()
            return 'opened'
        elif(time.time() - begin >= 180):
            count += 1
            return 'opened'
        else:
            return 'not_yet'

class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigating'])

    def execute(self, userdata):

        global goal_totoal, count

        # publish goal to tare
        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.pose.position.x = goal_totoal[count][0]
        pose.pose.position.y = goal_totoal[count][1]

        goal_pub.publish(pose)

        pub_info.publish("nav")
        return 'navigating'

class is_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_yet', 'navigated'])
        
        self.get_robot_pos = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    
    def execute(self, userdata):

        global success, begin
        global goal_totoal, count

        self.goal = np.array([goal_totoal[count][0], goal_totoal[count][1]]) 
        robot_pose = self.get_robot_pos("robot","")

        x, y = robot_pose.pose.position.x, robot_pose.pose.position.y
        dis = np.linalg.norm(self.goal - np.array([x, y]))

        if(dis < 0.8):
            pub_info.publish("stop")
            success += 1
            count += 1
            return 'navigated'
        elif(time.time() - begin >= 180):
            count += 1
            return 'navigated'
        else:
            return 'not_yet'

def main():

    rospy.init_node("doorgym_node", anonymous=False)

    sm = smach.StateMachine(outcomes=['end'])

    global method

    method = rospy.get_param("~method")

    with sm:
        smach.StateMachine.add('loop', loop(), transitions={'looping':'init', 'loop_done':'end'})
        smach.StateMachine.add('init', init(), transitions={'init_done':'nav_to_door'})
        smach.StateMachine.add('nav_to_door', nav_to_door(), transitions={'navigating':'reach_door'})
        smach.StateMachine.add('reach_door', reach_door(), transitions={'reached_door':'open', 'not_yet':'nav_to_door'})
        smach.StateMachine.add('open', open_door(), transitions={'opening':'is_open'})
        smach.StateMachine.add('is_open', is_open(), transitions={'opened':'nav_to_goal', 'not_yet':'open'})
        smach.StateMachine.add('nav_to_goal', Navigation(), transitions={'navigating':'is_goal'})
        smach.StateMachine.add('is_goal', is_goal(), transitions={'not_yet':'nav_to_goal', 'navigated':'loop'})

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