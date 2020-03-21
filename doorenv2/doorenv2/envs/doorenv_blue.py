import numpy as np
from gym import utils, spaces
from gym.envs.mujoco import mujoco_env
from gym.envs.robotics.rotations import quat2euler, euler2quat, mat2euler
import os
# import random
from random import uniform, randint, randrange
from mjremote import mjremote
import time
from doorenv2.envs.doorenv import DoorEnv

class DoorEnvBlueV1(DoorEnv, utils.EzPickle):
    def __init__(self,
                port=1050,
                unity=False,visionnet_input=False,
                world_path='/home/demo/DoorGym/world_generator/world/pull_floatinghook',
                pos_control=False
        ):
        super().__init__(
            port=port,
            unity=unity,
            visionnet_input=visionnet_input,
            world_path=world_path,
            pos_control=pos_control,
        )
        # self.port = port
        # self.hooked = True
        # self.untucked = True
        # self.init_done = False
        # self.pos_control = pos_control
        # self.hook_ratio = -1 #-1:all non_hooked, 100:all hooked
        # self.untucked_ratio = -1 #-1:all non-untucked, 100:all untucked
        # self.imgsize_h = 640
        # self.imgsize_w = 640
        # self.visionnet_input = visionnet_input
        # self.gripper_action = np.zeros(4)
        # self.xml_path = self.random_world(world_path)

        # if self.xml_path.find("float")>-1:
        #     if self.xml_path.find("hook")>-1:
        #         self.nn = 6
        #     if self.xml_path.find("gripper")>-1:
        #         self.nn = 7
        # else:
        #     if self.xml_path.find("mobile")>-1:
        #         if self.xml_path.find("hook")>-1:
        #             self.nn = 9
        #         if self.xml_path.find("gripper")>-1:
        #             self.nn = 10
        #     else:
        #         if self.xml_path.find("hook")>-1:
        #             self.nn = 7
        #         if self.xml_path.find("gripper")>-1:
        #             self.nn = 8

        # self.unity = unity
        # if self.visionnet_input:
        #     if self.unity:
        #         self.b = bytearray(3*self.imgsize_h*self.imgsize_w)
        #         self.no_viewer = False
        #     else:
        #         self.no_viewer = True
        # else:
        #     self.no_viewer = False
        # frame_skip = 20
        utils.EzPickle.__init__(self)
        # mujoco_env.MujocoEnv.__init__(self, self.xml_path, frame_skip)
        # gripper_space = self.gripper_action.shape[0]
        # if self.xml_path.find("gripper")>-1:
        #     bounds = self.model.actuator_ctrlrange.copy()
        #     low, high = bounds.T
        #     low, high = low[:-gripper_space], high[:-gripper_space] # four joints for finger is a dependant of finger inertial joint
        #     self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
        #     self.gripper_action = self.sim.data.qpos[-gripper_space:]
        # self.init_done = True
        # self.model_origin = self.model

    def gripper_action_gen(self, a):
            self.gripper_action = np.array([a[-1],-a[-1],a[-1],-a[-1]])
            return np.concatenate((a,self.gripper_action))

    def randomized_property(self):
        # import pprint as pp
        # pp.pprint(dir(self.model), width=1)
        # print(">>>>>before>>>>>>>")
        # pp.pprint(self.model.actuator_gainprm)

        self.model.body_mass[10:16] = self.sample_gaussiannormal(self.model_origin.body_mass[10:16], 0.2) # gaussiannormal x original_mass
        self.model.dof_damping[0:10] = self.sample_gaussiannormal(self.model_origin.dof_damping[0:10], 0.2) # gaussiannormal x original_damping
        self.model.actuator_gainprm[:,0] = self.sample_gaussiannormal(self.model_origin.actuator_gainprm[:,0], 0.1) # gaussiannormal x original_damping

        # self.model.body_mass[10:16] = self.sample_lognormal(self.model_origin.body_mass[10:16], 0.1, 0.4) # lognormal [0.4, 4.0] x original_mass
        # self.model.dof_damping[0:10] = self.sample_lognormal(self.model_origin.dof_damping[0:10], 0.3, 1.0) # lognormal [0.4, 20.0] x original_damping
        # self.model.actuator_gainprm[:,0] = self.sample_lognormal(self.model_origin.actuator_gainprm[:,0], 0.1, 0.2) # lognormal [0.5, 2.0] x original_damping

        # print(">>>>>after>>>>>>>")
        # pp.pprint(self.model.actuator_gainprm)

    def _reset_model(self, gg=2, hooked=False, untucked=False):
        qpos = self.init_qpos

        if self.xml_path.find("float")>-1:
            qpos = self.np_random.uniform(low=-0.3, high=0.3, size=self.model.nq) + self.init_qpos
            if self.xml_path.find("hook")>-1:
                qpos[self.nn-1] = np.random.uniform(0.0,3.13)
            if self.xml_path.find("gripper")>-1:
                qpos[self.nn-2] = np.random.uniform(0.0,3.13)
        elif self.xml_path.find("mobile")>-1:
            qpos[0] = 0.0 + uniform(-0.0, 0.0)           # x_slider
            qpos[1] = 0.0 + uniform(-0.0, -0.0)          # y_slider
            qpos[2] = 0.0 + uniform(-2.3412, 3.3999)     # base_roll_joint
            qpos[3] = 0.0 + uniform(-2.2944, 0)          # shoulder_lift_joint
            qpos[4] = 0.0 + uniform(-2.6761, 2.6761)     # shoulder_roll_joint
            qpos[5] = 1.0 + uniform(-2.2944, 0)          # elbow_lift_joint
            qpos[6] = 0.0 + uniform(-2.6761, 2.6761)     # elbow_roll_joint
            qpos[7] = 1.0 + uniform(-2.2944, 0)          # wrist_lift_joint
            qpos[8] = 0.0 + uniform(-2.6761, 2.6761)     # wrist_roll_joint
        else:
            qpos = self.init_qpos
            qpos[0] = 0.0 + uniform(-0.1, 0.1)     # base_roll_joint
            qpos[1] = 0.0 + uniform(-0.1, 0.1)     # shoulder_lift_joint
            qpos[2] = 0.0 + uniform(-0.1, 0.1)     # shoulder_roll_joint
            qpos[3] = 0.0 + uniform(-0.1, 0.1)     # elbow_lift_joint
            qpos[4] = 0.0 + uniform(-0.1, 0.1)     # elbow_roll_joint
            qpos[5] = 0.0 + uniform(-0.1, 0.1)     # wrist_lift_joint
            qpos[6] = 0.0 + uniform(-0.1, 0.1)     # wrist_roll_joint

        if self.xml_path.find("pull")>-1:
            self.goal = self.np_random.uniform(low=-.15, high=.15, size=gg)
            if self.xml_path.find("lefthinge")>-1:
                self.goal[0] = np.random.uniform(-0.15,0.05)
                self.goal[1] = np.random.uniform(-0.15,0.15)
            else:
                self.goal[0] = np.random.uniform(-0.05,0.15)
                self.goal[1] = np.random.uniform(-0.15,0.15)
        else:
            self.goal = np.zeros(gg)
            self.goal[0] = np.random.uniform(-0.15,0.15)

        qpos[self.nn:-gg] = 0
        qpos[-gg:] = self.goal
        # qvel = self.init_qvel
        # self.set_state(qpos, qvel)

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

        return self._get_obs()

    def get_robot_joints(self):
        return np.concatenate([
            self.sim.data.qpos.flat[:self.nn],
            self.sim.data.qvel.flat[:self.nn]])

    def get_finger_target(self):
        if self.xml_path.find("hook")>-1:
            return self.sim.data.get_geom_xpos("hookfinger_2")
        elif self.xml_path.find("gripper")>-1:
            return (self.sim.data.get_geom_xpos("fingerleft2") \
                + self.sim.data.get_geom_xpos("fingerright2"))/2.0
        else:
            assert "not sure about the end-effector type"

    def get_finger_ori(self):
        if self.xml_path.find("hook")>-1:
            return quat2euler(self.sim.data.get_body_xquat("robotfinger_hook_target"))
        elif self.xml_path.find("gripper")>-1:
            return quat2euler(self.sim.data.get_body_xquat("robotwrist_rolllink"))
        else:
            assert "not sure about the end-effector type"
    
    def get_finger_quat(self):
        if self.xml_path.find("hook")>-1:
            return self.sim.data.get_body_xquat("robotfinger_hook_target")
        elif self.xml_path.find("gripper")>-1:
            return self.sim.data.get_body_xquat("robotwrist_rolllink")
        else:
            assert "not sure about the end-effector type"


class DoorEnvBlueV2(DoorEnv, utils.EzPickle):
    def __init__(self,
                port=1050,
                unity=False,visionnet_input=False,
                world_path='/home/demo/DoorGym/world_generator/world/pull_floatinghook',
                pos_control=False
        ):
        super().__init__(
            port=port,
            unity=unity,
            visionnet_input=visionnet_input,
            world_path=world_path,
            pos_control=pos_control,
        )
        utils.EzPickle.__init__(self)

    def gripper_action_gen(self, a):
            self.gripper_action = np.array([a[-1],-a[-1],a[-1],-a[-1]])
            return np.concatenate((a,self.gripper_action))

    def physics_randomization(self):
        self.model.body_mass[1:18] = self.sample_gaussiannormal(self.model_origin.body_mass[1:18], 0.2) # gaussiannormal x original_mass
        self.model.dof_damping[0:12] = self.sample_gaussiannormal(self.model_origin.dof_damping[0:12], 0.2) # gaussiannormal x original_damping
        self.model.actuator_gainprm[:,0] = self.sample_gaussiannormal(self.model_origin.actuator_gainprm[:,0], 0.1) # gaussiannormal x original_damping

    def set_base_pos(self, pos_list=[0.6, 0.7, 0.7]):
        for i,x in enumerate(pos_list):
            self.model.body_pos[1,i] = x

    # def color_randomization(self):
        # import pprint as pp
        # import sys
        # pp.pprint(dir(self.model), width=1)
        # print(">>>>>before>>>>>>>")

        # pp.pprint(self.model.geom_rgba)
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
        # sys.exit(1)

    def _reset_model(self, gg=2, hooked=False, untucked=False):
        qpos = self.init_qpos
        qpos[0] = 0.0 + uniform(-0.1, 0.1)     # base_roll_joint
        qpos[1] = 0.0 + uniform(-0.1, 0.1)     # shoulder_lift_joint
        qpos[2] = 0.0 + uniform(-0.1, 0.1)     # shoulder_roll_joint
        qpos[3] = 0.0 + uniform(-0.1, 0.1)     # elbow_lift_joint
        qpos[4] = 0.0 + uniform(-0.1, 0.1)     # elbow_roll_joint
        qpos[5] = 0.0 + uniform(-0.1, 0.1)     # wrist_lift_joint
        qpos[6] = 0.0 + uniform(-0.1, 0.1)     # wrist_roll_joint

        if self.xml_path.find("pull")>-1:
            self.goal = self.np_random.uniform(low=-.15, high=.15, size=gg)
            if self.xml_path.find("lefthinge")>-1:
                self.goal[0] = np.random.uniform(-0.15,0.05)
                self.goal[1] = np.random.uniform(-0.15,0.15)
            else:
                self.goal[0] = np.random.uniform(-0.05,0.15)
                self.goal[1] = np.random.uniform(-0.15,0.15)
        else:
            self.goal = np.zeros(gg)
            self.goal[0] = np.random.uniform(-0.15,0.15)

        qpos[self.nn:-gg] = 0
        qpos[-gg:] = self.goal
        qvel = self.init_qvel 
        self.set_state(qpos, qvel)

        if self.unity:
            self.remote.setqpos(self.sim.data.qpos)
        return self._get_obs()

    def get_robot_joints(self):
        return np.concatenate([
            self.sim.data.qpos.flat[:self.nn],
            self.sim.data.qvel.flat[:self.nn]])

    def get_finger_target(self):
        return (self.sim.data.get_geom_xpos("fingerleft2") \
            + self.sim.data.get_geom_xpos("fingerright2"))/2.0

    def get_finger_ori(self):
        return quat2euler(self.sim.data.get_body_xquat("robotwrist_rolllink"))
    
    def get_finger_quat(self):
        return self.sim.data.get_body_xquat("robotwrist_rolllink")