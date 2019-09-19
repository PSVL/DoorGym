import numpy as np
from gym import utils, spaces
from gym.envs.mujoco import mujoco_env
from gym.envs.robotics.rotations import quat2euler, euler2quat, mat2euler
import os
import random
import torch
from mjremote import mjremote
import time
import matplotlib.pyplot as plt

class DoorEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    static_nn = 0
    def __init__(self, port=1050, unity=False, visionnet_input=False,\
                world_path='/u/home/urakamiy/doorgym/world_generator/world/pull_floatinghook'):
        self.tt = 0
        self.port = port
        self.hooked = True
        self.untucked = True
        self.first_img = None
        self.init_done = False
        self.hook_ratio = -1 #-1:all non_hooked, 100:all hooked
        self.untucked_ratio = -1 #-1:all non-untucked, 100:all untucked
        self.switch_avg = 0.0
        self.imgsize = 256
        self.visionnet_input = visionnet_input
        self.gripper_action = np.zeros(4)
        self.xml_path = self.random_world(world_path)

        if self.xml_path.find("baxter")>-1:
            if self.xml_path.find("botharm")>-1:
                self.nn = 16
                self.all_joints = self.nn + 4
            else:
                self.nn = 8
                self.all_joints = self.nn + 2
                self.gripper_action = np.zeros(2)
        else:
            if self.xml_path.find("float")>-1:
                if self.xml_path.find("hook")>-1:
                    self.nn = 6
                if self.xml_path.find("gripper")>-1:
                    self.nn = 7
            else:
                if self.xml_path.find("mobile")>-1:
                    if self.xml_path.find("hook")>-1:
                        self.nn = 9
                    if self.xml_path.find("gripper")>-1:
                        self.nn = 10
                else:
                    if self.xml_path.find("hook")>-1:
                        self.nn = 7
                    if self.xml_path.find("gripper")>-1:
                        self.nn = 8

        self.unity = unity
        if self.visionnet_input:
            if self.unity:
                self.b = bytearray(3*self.imgsize*self.imgsize)
                self.no_viewer = False
            else:
                self.no_viewer = True
        else:
            self.no_viewer = False
        frame_skip = 20
        utils.EzPickle.__init__(self)
        mujoco_env.MujocoEnv.__init__(self, self.xml_path, frame_skip)
        gripper_space = self.gripper_action.shape[0]
        # print("gripper space", gripper_space)
        if self.xml_path.find("gripper")>-1 or self.xml_path.find('baxter')>-1:
            bounds = self.model.actuator_ctrlrange.copy()
            low, high = bounds.T
            low, high = low[:-gripper_space], high[:-gripper_space] # four joints for finger is a dependant of finger inertial joint
            self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
            self.gripper_action = self.sim.data.qpos[-gripper_space:]
        self.init_done = True

    def __delete__(self):
        self.disconnet_to_unity()

    def close(self):
        self.disconnet_to_unity()

    def init(self, rank=0):
        self.unity_init(self.port + rank%8)
        self.change_model(self.xml_path)

    def step(self, a):
        # print("step")
        if not self.unity and self.no_viewer:
            print("made mujoco viewer")
            self.viewer = self._get_viewer('human')
            self.viewer_setup()
            self.no_viewer = False

        reward_dist = -np.linalg.norm(self.get_dist_vec())
        reward_log_dist = -np.log(np.square(np.linalg.norm(reward_dist))+5e-3) - 5.0 
        reward_ori = - np.linalg.norm(self.get_ori_diff_no_xaxis())
        reward_door = abs(self.sim.data.get_joint_qpos("hinge0")) * 30

        if self.xml_path.find("gripper")>-1:
            reward_ctrl = - np.mean(np.square(a[:-2]))
        else:
            reward_ctrl = - np.mean(np.square(a))

        if self.xml_path.find("lever")>-1 or self.xml_path.find("round")>-1:
            reward_doorknob = abs(self.sim.data.get_joint_qpos("hinge1")) * 50
            reward = reward_door + reward_doorknob + reward_ctrl + reward_ori + reward_dist + reward_log_dist
        else:
            reward = reward_door + reward_ctrl + reward_ori + reward_dist + reward_log_dist

        if self.init_done:
            if self.xml_path.find("gripper")>-1:
                self.gripper_action = np.array([a[-1],-a[-1],a[-1],-a[-1]])
                a = np.concatenate((a,self.gripper_action))
            elif self.xml_path.find("baxter")>-1:
                if self.xml_path.find("both")>-1:
                    self.gripper_action = np.array([-a[-9],a[-9],-a[-1],a[-1]])
                else:
                    self.gripper_action = np.array([-a[-1],a[-1]])
                a = np.concatenate((a,self.gripper_action))
            else:
                pass

        self.do_simulation(a, self.frame_skip)
        ob = self._get_obs()
        done = False

        if self.init_done and self.unity:
            self.remote.setqpos(self.sim.data.qpos)

        self.tt += 1
        return ob, reward, done, dict(reward_dist=reward_dist, reward_ctrl=reward_ctrl)

    def reset_model(self, gg=2):
        # self.hooked = True
        hooked_chance = np.random.randint(100)
        untucked_chance = np.random.randint(100)

        if hooked_chance>=self.hook_ratio:
            self.hooked = False
        if untucked_chance>=self.untucked_ratio:
            self.untucked = False
        return self._reset_model(gg=gg, hooked=self.hooked, untucked=self.untucked)

    def _reset_model(self, gg=2, hooked=False, untucked=False):
        
        if self.xml_path.find('baxter')>-1:
            if untucked:
                # print("init qpos",self.init_qpos.shape)
                # print("init qvel",self.init_qvel.shape)
                qpos = self.init_qpos
                # qpos[:20] = np.array([0.08, -1.00, 1.19, 1.94, -0.67, 1.03, 0.5, 0.02, 0.0, 0.0,\
                #                 -0.08, -1.00, -1.19, 1.94, 0.67, 1.03, -0.5, 0.0, 0.0, 0.0])
                qpos[:20] = np.array([0.08, -1.00, 1.19, 1.94, -0.67, 1.03, 0.5, 0.02, 0.0, 0.0,\
                    1.15, 1.05, -0.10, 0.50, -1.00, 0.01, -1.92, 0.0, 0.0, 0.0])
                # print("this qpos", qpos.shape)
            else:
                qpos = self.init_qpos
                # if self.xml_path.find('both')>-1:
                qpos[0] = random.uniform(-1.65, 1.65)    # right_s0
                qpos[1] = random.uniform(-2.10, 1.00)    # right_s1
                qpos[2] = random.uniform(-3.00, 3.00)    # right_e0
                qpos[3] = random.uniform(-0.05, 2.50)    # right_e1
                qpos[4] = random.uniform(-3.00, 3.00)    # right_w0
                qpos[5] = random.uniform(-1.55, 2.00)       # right_w1
                qpos[6] = random.uniform(-3.00, 3.00)        # right_w2
                qpos[7] = random.uniform(0, 0.02)          # robotfinger_actuator_joint_r
                qpos[8] = random.uniform(-0.02, 0)            # r_gripper_l_finger_joint
                qpos[9] = random.uniform( 0, 0.02)            # r_gripper_r_finger_joint

                # qpos[0] =  -1.15 + random.uniform(-0.1, 0.1)    # right_s0
                # qpos[1] =   1.05 + random.uniform(-0.1, 0.1)        # right_s1
                # qpos[2] =   0.10 + random.uniform(-0.1, 0.1)    # right_e0
                # qpos[3] =   0.50 + random.uniform(-0.1, 0.1)         # right_e1
                # qpos[4] =   1.00 + random.uniform(-0.1, 0.1)        # right_w0
                # qpos[5] =  -0.01 + random.uniform(-0.1, 0.1)       # right_w1
                # qpos[6] =   1.92 + random.uniform(-0.1, 0.1)        # right_w2
                # qpos[7] =   0.00 + random.uniform(0, 0.020833)          # robotfinger_actuator_joint_r
                # qpos[8] =   0.00 + random.uniform(-0.02, 0)            # r_gripper_l_finger_joint
                # qpos[9] =   0.00 + random.uniform( 0, 0.02)            # r_gripper_r_finger_joint

                qpos[10] =  1.15 #+ random.uniform(-0.2, 0.2)   # left_s0
                qpos[11] =  1.05 #+ random.uniform(-0.2, 0.2)   # left_s1
                qpos[12] = -0.10 #+ random.uniform(-0.2, 0.2)   # left_e0
                qpos[13] =  0.50 #+ random.uniform(-0.2, 0.2)   # left_e1
                qpos[14] = -1.00 #+ random.uniform(-0.2, 0.2)   # left_w0
                qpos[15] = -0.01 #+ random.uniform(-0.2, 0.2)   # left_w1
                qpos[16] = -1.92 #+ random.uniform(-0.2, 0.2)   # left_w2
                qpos[17] =  0.00 #+ random.uniform( 0, 0.02)    # robotfinger_actuator_joint_l
                qpos[18] =  0.00 #+ random.uniform(-0.02, 0)    # l_gripper_l_finger_joint
                qpos[19] =  0.00 #+ random.uniform( 0, 0.02)    # l_gripper_r_finger_joint
        elif self.xml_path.find("float")>-1:
            qpos = self.np_random.uniform(low=-0.3, high=0.3, size=self.model.nq) + self.init_qpos
            if self.xml_path.find("hook")>-1:
                qpos[self.nn-1] = np.random.uniform(0.0,3.13)
            if self.xml_path.find("gripper")>-1:
                qpos[self.nn-2] = np.random.uniform(0.0,3.13)
        elif self.xml_path.find("mobile")>-1:
            qpos = self.init_qpos
            qpos[0] = 0.0 + random.uniform(-0.0, 0.0)           # x_slider
            qpos[1] = 0.0 + random.uniform(-0.0, -0.0)          # y_slider
            qpos[2] = 0.0 + random.uniform(-2.3412, 3.3999)     # base_roll_joint
            qpos[3] = 0.0 + random.uniform(-2.2944, 0)          # shoulder_lift_joint
            qpos[4] = 0.0 + random.uniform(-2.6761, 2.6761)     # shoulder_roll_joint
            qpos[5] = 1.0 + random.uniform(-2.2944, 0)          # elbow_lift_joint
            qpos[6] = 0.0 + random.uniform(-2.6761, 2.6761)     # elbow_roll_joint
            qpos[7] = 1.0 + random.uniform(-2.2944, 0)          # wrist_lift_joint
            qpos[8] = 0.0 + random.uniform(-2.6761, 2.6761)     # wrist_roll_joint
        else:
            qpos = self.init_qpos
            qpos[0] = 0.0 + random.uniform(-1.57, 1.57)         # base_roll_joint
            qpos[1] = -1.57 + random.uniform(-0.3, 0.3)         # shoulder_lift_joint
            qpos[2] = 0.0 + random.uniform(-2.6761, 2.6761)     # shoulder_roll_joint
            qpos[3] = 1.0 + random.uniform(-0.6, 0)             # elbow_lift_joint
            qpos[4] = 0.0 + random.uniform(-2.6761, 2.6761)     # elbow_roll_joint
            qpos[5] = 1.0 + random.uniform(-0.6, 0)             # wrist_lift_joint
            qpos[6] = 0.0 + random.uniform(-2.6761, 2.6761)     # wrist_roll_joint

        if self.xml_path.find("pull")>-1:
            self.goal = self.np_random.uniform(low=-.0, high=-.15, size=gg)
            # self.goal = self.np_random.uniform(low=-.15, high=-.15, size=gg)
            if self.xml_path.find("lefthinge")>-1:
                self.goal[0] = np.random.uniform(-0.15,0.05)
            else:
                # print("not right or left")
                # self.goal[0] = np.random.uniform(-0.00,0.15)
                self.goal[0] = np.random.uniform(-0.10,0.15)
        else:
            self.goal = np.zeros(gg)

            self.goal[0] = np.random.uniform(-0.15,0.15)

        # if self.xml_path.find('baxter')>-1 and not self.xml_path.find('both')>-1:
        #     qpos[(self.all_joints*2):-gg] = 0
        # elif self.xml_path.find('baxter')>-1:
        #     qpos[self.all_joints:-gg] = 0
        # else:
        #     qpos[self.nn:-gg] = 0

        if self.xml_path.find('baxter')>-1:
            if self.xml_path.find('both')>-1:
                qpos[self.all_joints:-gg] = 0
            else:
                qpos[(self.all_joints*2):-gg] = 0
        else:
            qpos[self.nn:-gg] = 0

        qpos[-gg:] = self.goal

        # print("qpos:", qpos, " gg:", self.goal)

        qvel = self.init_qvel
        self.set_state(qpos, qvel)

        if hooked:
            if self.xml_path.find("float")>-1:
                robot_origin = np.array([1.0, 0, 1.2])
                if self.xml_path.find("lever")>-1:
                    goal_in_xyz = self.sim.data.get_geom_xpos("door_knob_4") - robot_origin
                    offset_to_hook = np.array([0.13,0.0,0.0])
                elif self.xml_path.find("round")>-1:
                    goal_in_xyz = self.sim.data.get_geom_xpos("door_knob_2") - robot_origin
                    offset_to_hook = np.array([0.0,0.0,0.0]) 
                elif self.xml_path.find("pull")>-1:
                    goal_in_xyz = self.sim.data.get_geom_xpos("door_knob_7") - robot_origin
                    offset_to_hook = np.array([0.13,0.0,0.0]) 
                else:
                    assert "not sure about the door knob type"

                if self.xml_path.find("hook")>-1:
                    offset_to_hook_randomness = np.array([np.random.uniform(-0.01,0.01), np.random.uniform(-0.005,0.005), np.random.uniform(-0.06,0.06)])
                    hand_init_pos_3D = goal_in_xyz + offset_to_hook + offset_to_hook_randomness
                
                    hand_ori_random = self.np_random.uniform(low=-0.05, high=0.05, size=3)
                    wrist_dir_chance = np.random.randint(100)
                    if wrist_dir_chance>=50:
                        hand_ori_random[-1] = np.random.uniform(0.0,0.4)
                    else:
                        hand_ori_random[-1] = np.random.uniform(2.74,3.14)
                    qpos[:self.nn] = np.concatenate((hand_init_pos_3D,hand_ori_random))

                if self.xml_path.find("gripper")>-1:
                    offset_to_hook_randomness = np.array([0.0, 0.0, np.random.uniform(-0.06,0.06)])
                    hand_init_pos_3D = goal_in_xyz + offset_to_hook + offset_to_hook_randomness
                
                    hand_ori_random = self.np_random.uniform(low=-0.01, high=0.01, size=3)
                    wrist_dir_chance = np.random.randint(100)
                    if wrist_dir_chance>=50:
                        hand_ori_random[-1] = np.random.uniform(0.0,0.01)
                    else:
                        hand_ori_random[-1] = np.random.uniform(3.13,3.14)
                    qpos[:self.nn-1] = np.concatenate((hand_init_pos_3D,hand_ori_random))
                    qpos[0] -= 0.02
                    qpos[self.nn: self.nn+4] = np.array([1.0,-1.0,1.0,-1.0]) 
            
                qvel = self.init_qvel 
                self.set_state(qpos, qvel)

        if self.unity:
            self.remote.setqpos(self.sim.data.qpos)

        self.tt = 0
        return self._get_obs()

    def _get_obs(self):
        # print("finger",self.get_finger_target())
        # print("door",self.get_knob_target())
        if self.visionnet_input:
            return np.concatenate([
                self.get_robot_joints(),
                self.get_finger_target(), # if knob dist from img
                self.get_flatten_img(cam_num=1), 
                self.get_flatten_img(cam_num=2) 
                ])
        else:
            # print("qpos", self.sim.data.qpos)
            return np.concatenate([
                self.get_robot_joints(),
                self.get_dist_vec() # if knob dist from mujoco
                ])

    def connect_to_unity(self):
        while True:
            try:
                self.remote.connect(port=self.port)
                print("Size of qpos:", self.remote.nqpos,", Size of mocap:", self.remote.nmocap, \
                                    ", No. of camera:", self.remote.ncamera,", Size of image w=",\
                                                    self.remote.width, ",h=", self.remote.height)
                break
            except:
                print("connect waiting")
                time.sleep(1)
    
    def disconnet_to_unity(self):
        self.remote.close()

    def unity_init(self, port):
        print("making unity remote connecting to {}".format(port))
        self.port = port
        self.remote = mjremote()
        self.change_model(self.xml_path)
        self.remote.setcamera(0)
      
    def change_model(self, full_path):
        print('Setting world in unity')
        # print("full path", full_path)
        self.remote.changeworld(full_path)
        while True:
            try:
                self.remote.connect(port=self.port)
                print("Size of qpos:", self.remote.nqpos,", Size of mocap:", self.remote.nmocap, \
                    ", No. of camera:", self.remote.ncamera,", Size of image w=",\
                                    self.remote.width, ",h=", self.remote.height)
                time.sleep(0.5)
                break
            except:
                print("now connecting")
                time.sleep(1)

    def random_world(self, world_path):
        self.world = random.choice(os.listdir(world_path))
        print("world:", self.world)
        xml_path = os.path.join(world_path, self.world)
        return xml_path       

    def normalizer(self, img):
        mean = [0.5, 0.5, 0.5]
        std = [0.5, 0.5, 0.5]
        img = np.array(img).astype(np.float32)
        img /= 255.0
        img -= mean
        img /= std
        return img

    def get_flatten_img(self, cam_num, device=0):
        if self.unity:
            if self.init_done:
                self.remote.setcamera(1)
                time.sleep(0.05)
                self.remote.getimage(self.b)
                img = np.reshape(self.b, (self.imgsize, self.imgsize, 3))
            else:
                img = np.reshape(self.b, (self.imgsize, self.imgsize, 3))
        else:
            img = self.sim.render(width=self.imgsize,
                                  height=self.imgsize,
                                  mode='offscreen',
                                  camera_name="camera{}".format(cam_num))
        img = img[::-1,:,:]

        img = self.normalizer(img)
        img = np.transpose(img, (2,0,1))
        img = np.reshape(img, (3*self.imgsize*self.imgsize))
        return img

    def get_robot_joints(self):
        if self.xml_path.find("baxter")>-1:
            if self.xml_path.find("leftarm")>-1:
                # print("baxter qpos",self.sim.data.qpos)
                # print("baxter left obs", self.sim.data.qpos.flat[self.all_joints:self.all_joints+self.nn])
                return np.concatenate([
                    self.sim.data.qpos.flat[self.all_joints:self.all_joints+self.nn],
                    self.sim.data.qvel.flat[self.all_joints:self.all_joints+self.nn]])
        return np.concatenate([
            self.sim.data.qpos.flat[:self.nn],
            self.sim.data.qvel.flat[:self.nn]])

    def get_knob_target(self):
        # print("head camera pos:",self.sim.data.get_body_xpos("head_camera"))
        if self.xml_path.find("lever")>-1:
            return self.sim.data.get_geom_xpos("door_knob_4")
        elif self.xml_path.find("round")>-1:
            return self.sim.data.get_geom_xpos("door_knob_2")
        elif self.xml_path.find("pull")>-1:
            knob_upper = self.sim.data.get_geom_xpos("door_knob_5")
            knob_lower = self.sim.data.get_geom_xpos("door_knob_8")
            return (knob_upper+knob_lower)/2.0
        else:
            assert "not sure about the door knob type"

    def get_finger_target(self):
        if self.xml_path.find("hook")>-1:
            return self.sim.data.get_geom_xpos("hookfinger_2")
        elif self.xml_path.find("gripper")>-1:
            return (self.sim.data.get_geom_xpos("fingerleft2") \
                + self.sim.data.get_geom_xpos("fingerright2"))/2.0
        elif self.xml_path.find("baxter")>-1:
            if self.xml_path.find("leftarm")>-1:
                return (self.sim.data.get_body_xpos("l_gripper_l_finger_tip") \
                    + self.sim.data.get_body_xpos("l_gripper_r_finger_tip"))/2.0
            else:
                return (self.sim.data.get_body_xpos("r_gripper_l_finger_tip") \
                    + self.sim.data.get_body_xpos("r_gripper_r_finger_tip"))/2.0
        else:
            assert "not sure about the end-effector type"

    def get_dist_vec(self):
        # print("finger pos", self.get_finger_target(), "knob pos", self.get_knob_target())
        return self.get_finger_target() - self.get_knob_target()

    def get_finger_ori(self):
        if self.xml_path.find("hook")>-1:
            return quat2euler(self.sim.data.get_body_xquat("robotfinger_hook_target"))
        elif self.xml_path.find("gripper")>-1:
            return quat2euler(self.sim.data.get_body_xquat("robotwrist_rolllink"))
        elif self.xml_path.find("baxter")>-1:
            if self.xml_path.find("leftarm")>-1:
                return quat2euler(self.sim.data.get_body_xquat("left_wrist"))
            else:
                return quat2euler(self.sim.data.get_body_xquat("right_wrist"))
        else:
            assert "not sure about the end-effector type"
    
    def get_finger_quat(self):
        if self.xml_path.find("hook")>-1:
            return self.sim.data.get_body_xquat("robotfinger_hook_target")
        elif self.xml_path.find("gripper")>-1:
            return self.sim.data.get_body_xquat("robotwrist_rolllink")
        elif self.xml_path.find("baxter")>-1:
            if self.xml_path.find("leftarm")>-1:
                return self.sim.data.get_body_xquat("left_wrist")
            else:
                return self.sim.data.get_body_xquat("right_wrist")
        else:
            assert "not sure about the end-effector type"

    def get_ori_diff(self):
        return self.get_finger_ori() - quat2euler(self.sim.data.get_body_xquat("knob_link"))

    def get_ori_diff_no_xaxis(self):
        ori_diff = self.get_ori_diff()
        ori_diff[0] = 0 # ignore the xaxis rotation
        return ori_diff

    def reward_hook_dir(self):
        xaxis = quat2euler(self.sim.data.get_body_xquat("robotfinger_hook_target"))[0]
        vec = self.get_dist_vec()
        # higher reward if hook is heading in right direction 
        if vec[1]>=0: #hand on right side of knob (y-axis). hook x-axis has to be 0
            reward_hookdir = 1.57 - xaxis
        else: 
            reward_hookdir = xaxis - 1.57
        return reward_hookdir

    def viewer_setup(self, camera_type='global_cam', camera_select=1):
        if camera_type == 'fixed_cam':
            cam_type = const.CAMERA_FIXED
            camera_select = camera_select
        elif camera_type == 'global_cam':
            cam_type = 0
        DEFAULT_CAMERA_CONFIG = {
        'distance': 1.75,
        'azimuth': 245.0,
        'elevation': -13.0,
        'type': cam_type,
        'fixedcamid': camera_select
        }

        # self.viewer._run_speed = 0.075

        # print(self.viewer._run_speed)

        for key, value in DEFAULT_CAMERA_CONFIG.items():
            if isinstance(value, np.ndarray):
                getattr(self.viewer.cam, key)[:] = value
            else:
                setattr(self.viewer.cam, key, value)  