import numpy as np
from gym import utils, spaces
from gym.envs.mujoco import mujoco_env
from gym.envs.robotics.rotations import quat2euler, euler2quat, mat2euler
import os
import random
from random import uniform, randrange
from mjremote import mjremote
import time

class DoorEnv(mujoco_env.MujocoEnv):
    def __init__(self,
                port=1050,
                unity=False,
                visionnet_input=False,
                vision_obs= False,
                world_path='/home/demo/DoorGym/world_generator/world/pull_floatinghook',
                pos_control=False,
                ik_control=False,
                imgsize_h=640,
                imgsize_w=640):
        # print("passed",imgsize_h)
        self.port = port
        self.hooked = True
        self.untucked = True
        self.init_done = False
        self.pos_control = pos_control
        self.ik_control = ik_control
        self.hook_ratio = -1 #-1:all non_hooked, 100:all hooked
        self.untucked_ratio = -1 #-1:all non-untucked, 100:all untucked
        self.imgsize_h = imgsize_h
        self.imgsize_w = imgsize_w
        self.visionnet_input = visionnet_input
        self.vision_obs = vision_obs
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
        if self.visionnet_input or self.vision_obs:
            if self.unity:
                self.b = bytearray(3*self.imgsize_h*self.imgsize_w)
                self.no_viewer = False
            else:
                self.no_viewer = True
        else:
            self.no_viewer = False
        frame_skip = 20
        # utils.EzPickle.__init__(self)
        mujoco_env.MujocoEnv.__init__(self, self.xml_path, frame_skip)
        gripper_space = self.gripper_action.shape[0]
        # print("gripper space", gripper_space)
        if self.xml_path.find("gripper")>-1:
            bounds = self.model.actuator_ctrlrange.copy()
            low, high = bounds.T
            low, high = low[:-gripper_space], high[:-gripper_space] # four joints for finger is a dependant of finger inertial joint
            self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
            self.gripper_action = self.sim.data.qpos[-gripper_space:]
        self.init_done = True
        self.model_origin = self.model

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
        reward_door = abs(self.sim.data.get_joint_qpos("hinge0")) *30 #*30

        if self.xml_path.find("gripper")>-1:
            reward_ctrl = - np.mean(np.square(a[:-2]))
        else:
            reward_ctrl = - np.mean(np.square(a))

        if self.pos_control:
            reward_ctrl = 0

        if self.xml_path.find("lever")>-1 or self.xml_path.find("round")>-1:
            reward_doorknob = abs(self.sim.data.get_joint_qpos("hinge1")) * 50 #*50
            reward = reward_door + reward_doorknob + reward_ctrl + reward_ori + reward_dist + reward_log_dist
        else:
            reward = reward_door + reward_ctrl + reward_ori + reward_dist + reward_log_dist

        if self.init_done:
            if self.xml_path.find("gripper")>-1:
                a = self.gripper_action_gen(a)

        self.do_simulation(a, self.frame_skip)
        ob = self._get_obs()
        done = False

        if self.init_done and self.unity:
            self.remote.setqpos(self.sim.data.qpos)

        return ob, reward, done, dict(reward_dist=reward_dist, reward_ctrl=reward_ctrl)

    def gripper_action_gen(self, a):
        raise NotImplementedError()

    def reset_model(self, gg=2):
        property_DR = False
        if property_DR and self.init_done:
            self.physics_randomization()

        light_DR = False
        if light_DR:
            self.light_randomization()

        camera_DR = False
        if camera_DR:
            self.camera_randomization()

        color_DR = False
        if color_DR:
            self.color_randomization()

        mesh_DR = False
        if mesh_DR:
            self.mesh_randomization()

        set_base_pos = True
        if set_base_pos:
            self.set_base_pos()

        hooked_chance = np.random.randint(100)
        untucked_chance = np.random.randint(100)

        if hooked_chance>=self.hook_ratio:
            self.hooked = False
        if untucked_chance>=self.untucked_ratio:
            self.untucked = False
        return self._reset_model(gg=gg, hooked=self.hooked, untucked=self.untucked)

    def physics_randomization(self):
        raise NotImplementedError()

    def light_randomization(self):       
        light_n = self.model.light_pos.shape[0]
        light_pos, light_dir, light_diffuse = [], [], []
        for i in range(light_n):
            light_pos.append([randrange(0,500)/100.0, randrange(-500,500)/100.0, randrange(300,700)/100.0])
            light_dir.append([randrange(-50,50)/100.0, randrange(-50,50)/100.0, randrange(-50,-25)/100.0])
            light_diffuse.append([randrange(9,11)/10, randrange(9,11)/10, randrange(9,11)/10])
        self.model.light_pos[:,:] = np.array(light_pos)
        self.model.light_dir[:,:] = np.array(light_dir)
        self.model.light_diffuse[:,:] = np.array(light_diffuse)

    def camera_randomization(self):        
        cam_pos = [[0.99, 0.5, 1.0], [0.5, 0.0, 1.99]] #1m x 1m
        cam_ori = [[0.0, 1.57, 1.57], [0, 0, 0]]
        cam_fovy = [60, 60]
        for i in range(len(cam_pos)):
            for j in range(len(cam_pos[0])):
                cam_pos[i][j] += randrange(-15,15)/1000
        ori_dim = len(cam_ori[0])
        for i in range(len(cam_ori)):
            for j in range(ori_dim):
                cam_ori[i][j] += randrange(-17,17)/1000
        for i in range(len(cam_fovy)):
            cam_fovy[i] += randrange(-100,100)/100
        self.model.cam_pos[:,:] = np.array(cam_pos)
        self.model.cam_quat[:,:] = np.array(euler2quat(cam_ori))
        self.model.cam_fovy[:] = np.array(cam_fovy)
    
    def color_randomization(self):
        geom_n = self.model.geom_rgba.shape[0]
        geom_rgba = []
        for i in range(geom_n):
            geom_rgba.append([randrange(1,100)/100.0, randrange(1,100)/100.0, randrange(1,100)/100.0, 1.0])
        self.model.geom_rgba[:,:] = np.array(geom_rgba)

    def mesh_randomization(self):
        import pprint as pp
        import sys
        pp.pprint(dir(self.model), width=1)
        pp.pprint(dir(self.data), width=1)
        print(">>>>>before>>>>>>>")

        print("mesh face: ",self.model.mesh_face.shape)
        print("mesh faceadr: ",self.model.mesh_faceadr.shape)
        print("mesh facenum: ",self.model.mesh_facenum.shape)
        print("mesh graph: ",self.model.mesh_graph.shape)
        print("mesh graphadr: ",self.model.mesh_graphadr.shape)
        print("mesh names: ",len(self.model.mesh_names))
        print("mesh normal: ",self.model.mesh_normal.shape)
        print("mesh vert: ",self.model.mesh_vert.shape)

        # geom_n = self.model.geom_rgba.shape[0]

        # geom_rgba = []
        # for i in range(geom_n):
        #     geom_rgba.append([randrange(1,100)/100.0, randrange(1,100)/100.0, randrange(1,100)/100.0, 1.0])

        # self.model.geom_rgba[:,:] = np.array(geom_rgba)
        # self.model.cam_quat[:,:] = np.array(euler2quat(cam_ori))
        # self.model.cam_fovy[:] = np.array(cam_fovy)
        # print(">>>>>after>>>>>>>")
        # pp.pprint(self.model.geom_rgba)
        # pp.pprint(self.model.cam_quat)
        # pp.pprint(self.model.cam_fovy)
        sys.exit(1)

    def sample_gaussiannormal(self, property_array, sigma):
        shape = property_array.shape
        gaussian_sample = np.random.normal(1.0, sigma, shape)
        return gaussian_sample*property_array

    def sample_lognormal(self, property_array, mu, sigma):
        shape = property_array.shape
        log_sample = np.random.lognormal(mu, sigma, shape)
        return log_sample*property_array

    def _reset_model(self, gg=2, hooked=False, untucked=False):
        raise NotImplementedError()

    def _get_obs(self):
        if self.visionnet_input:
            return np.concatenate([
                self.get_robot_joints(),
                self.get_finger_target(), # if knob dist from img
                self.get_flatten_img(cam_num=1), 
                self.get_flatten_img(cam_num=2) 
                ])
        elif self.vision_obs:
            return np.concatenate([
                self.get_img(cam_num=2),
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
        if hasattr(self,'remote'):
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

    def get_img(self, cam_num, device=0):
        if self.unity:
            if self.init_done:
                self.remote.setcamera(1)
                time.sleep(0.05)
                self.remote.getimage(self.b)
                img = np.reshape(self.b, (self.imgsize_h, self.imgsize_w, 3))
            else:
                img = np.reshape(self.b, (self.imgsize_h, self.imgsize_w, 3))
        else:
            img = self.sim.render(width=self.imgsize_h,
                                  height=self.imgsize_w,
                                  mode='offscreen',
                                  camera_name="camera{}".format(cam_num))
        img = img[::-1,:,:]

        # print(img.shape)
        # import matplotlib.pyplot as plt
        # plt.imshow(img)
        # plt.show()
        # import sys
        # sys.exit(1)
        
        img = np.transpose(img, (2,0,1))
        return img

    def get_flatten_img(self, cam_num, device=0):
        if self.unity:
            if self.init_done:
                self.remote.setcamera(1)
                time.sleep(0.05)
                self.remote.getimage(self.b)
                img = np.reshape(self.b, (self.imgsize_h, self.imgsize_w, 3))
            else:
                img = np.reshape(self.b, (self.imgsize_h, self.imgsize_w, 3))
        else:
            img = self.sim.render(width=self.imgsize_h,
                                  height=self.imgsize_w,
                                  mode='offscreen',
                                  camera_name="camera{}".format(cam_num))
        img = img[::-1,:,:]

        img = self.normalizer(img)
        img = np.transpose(img, (2,0,1))
        img = np.reshape(img, (3*self.imgsize_h*self.imgsize_w))
        return img

    def get_doorangle(self):
        return self.sim.data.get_joint_qpos("hinge0")

    def get_robot_joints(self):
        raise NotImplementedError()

    def get_robot_joints_pos(self):
        return self.sim.data.qpos.flat[:self.nn]

    def get_knob_target(self):
        # print("head camera pos:",self.sim.data.get_body_xpos("head_camera"))
        if self.xml_path.find("lever")>-1:
            return self.sim.data.get_geom_xpos("door_knob_4")
        elif self.xml_path.find("round")>-1:
            return self.sim.data.get_geom_xpos("door_knob_2")
        elif self.xml_path.find("pull")>-1:
            knob_upper = self.sim.data.get_geom_xpos("door_knob_2")
            knob_lower = self.sim.data.get_geom_xpos("door_knob_3")
            return (knob_upper+knob_lower)/2.0
        else:
            assert "not sure about the door knob type"

    def get_finger_target(self):
        raise NotImplementedError()

    def get_base_pos(self):
        raise NotImplementedError()

    def get_dist_vec(self):
        return self.get_finger_target() - self.get_knob_target()

    def get_finger_ori(self):
        raise NotImplementedError()
    
    def get_finger_quat(self):
        raise NotImplementedError()

    def get_finger_vel(self):
        raise NotImplementedError()

    def get_finger_angvel(self):
        raise NotImplementedError()

    def get_gripper_pos(self):
        raise NotImplementedError()

    def get_ori_diff(self):
        return self.get_finger_ori() - quat2euler(self.sim.data.get_body_xquat("knob_link"))

    def get_ori_diff_no_xaxis(self):
        ori_diff = self.get_ori_diff()
        ori_diff[0] = 0 # ignore the xaxis rotation
        return ori_diff

    # def reward_hook_dir(self):
    #     xaxis = quat2euler(self.sim.data.get_body_xquat("robotfinger_hook_target"))[0]
    #     vec = self.get_dist_vec()
    #     # higher reward if hook is heading in right direction 
    #     if vec[1]>=0: #hand on right side of knob (y-axis). hook x-axis has to be 0
    #         reward_hookdir = 1.57 - xaxis
    #     else: 
    #         reward_hookdir = xaxis - 1.57
    #     return reward_hookdir

    def viewer_setup(self, camera_type='global_cam', camera_select=1):
        if camera_type == 'fixed_cam':
            cam_type = const.CAMERA_FIXED
            camera_select = camera_select
        elif camera_type == 'global_cam':
            cam_type = 0
        DEFAULT_CAMERA_CONFIG = {
        'distance': 3.50,
        'azimuth': 245.0,
        'elevation': -13.0,
        'type': cam_type,
        'fixedcamid': camera_select
        }

        self.viewer._run_speed = 0.075

        print(self.viewer._run_speed)

        for key, value in DEFAULT_CAMERA_CONFIG.items():
            if isinstance(value, np.ndarray):
                getattr(self.viewer.cam, key)[:] = value
            else:
                setattr(self.viewer.cam, key, value)  