import numpy as np
from gym import utils, spaces
from gym.envs.mujoco import mujoco_env
from gym.envs.robotics.rotations import quat2euler, euler2quat, mat2euler
import os
import random
from mjremote import mjremote
import time
from doorenv2.envs.doorenv import DoorEnv

class DoorEnvBaxter(DoorEnv, utils.EzPickle):
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

        # if self.xml_path.find("botharm")>-1:
        #     self.nn = 16
        #     self.all_joints = self.nn + 4
        # else:
        #     self.nn = 8
        #     self.all_joints = self.nn + 2
        #     self.gripper_action = np.zeros(2)

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
        # # print("gripper space", gripper_space)
        # if self.xml_path.find("gripper")>-1 or self.xml_path.find('baxter')>-1:
        #     bounds = self.model.actuator_ctrlrange.copy()
        #     low, high = bounds.T
        #     low, high = low[:-gripper_space], high[:-gripper_space] # four joints for finger is a dependant of finger inertial joint
        #     self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
        #     self.gripper_action = self.sim.data.qpos[-gripper_space:]
        # self.init_done = True
        # self.model_origin = self.model
        # print("its me mario")

    def gripper_action_gen(self, a):
        if self.xml_path.find("both")>-1:
            self.gripper_action = np.array([-a[-9],a[-9],-a[-1],a[-1]])
        else:
            self.gripper_action = np.array([-a[-1],a[-1]])
        a = np.concatenate((a,self.gripper_action))

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
        
        if untucked:
            # print("untucked position")
            qpos[:20] = np.array([0.08, -1.00, 1.19, 1.94, -0.67, 1.03, 0.5, 0.02, 0.0, 0.0,\
                1.15, 1.05, -0.10, 0.50, -1.00, 0.01, -1.92, 0.0, 0.0, 0.0])
        else:
            # print("randomized position")
            # qpos[0] = random.uniform(-1.65, 1.65)    # right_s0
            # qpos[1] = random.uniform(-2.10, 1.00)    # right_s1
            # qpos[2] = random.uniform(-3.00, 3.00)    # right_e0
            # qpos[3] = random.uniform(-0.05, 2.50)    # right_e1
            # qpos[4] = random.uniform(-3.00, 3.00)    # right_w0
            # qpos[5] = random.uniform(-1.55, 2.00)       # right_w1
            # qpos[6] = random.uniform(-3.00, 3.00)        # right_w2
            # qpos[7] = random.uniform(0, 0.02)          # robotfinger_actuator_joint_r
            # qpos[8] = random.uniform(-0.02, 0)            # r_gripper_l_finger_joint
            # qpos[9] = random.uniform( 0, 0.02)            # r_gripper_r_finger_joint

            qpos[0] =  -1.15 + random.uniform(-0.1, 0.1)    # right_s0
            qpos[1] =   1.05 + random.uniform(-0.1, 0.1)        # right_s1
            qpos[2] =   0.10 + random.uniform(-0.1, 0.1)    # right_e0
            qpos[3] =   0.50 + random.uniform(-0.1, 0.1)         # right_e1
            qpos[4] =   1.00 + random.uniform(-0.1, 0.1)        # right_w0
            qpos[5] =  -0.01 + random.uniform(-0.1, 0.1)       # right_w1
            qpos[6] =   1.92 + random.uniform(-0.1, 0.1)        # right_w2
            qpos[7] =   0.00 + random.uniform(0, 0.020833)          # robotfinger_actuator_joint_r
            qpos[8] =   0.00 + random.uniform(-0.02, 0)            # r_gripper_l_finger_joint
            qpos[9] =   0.00 + random.uniform( 0, 0.02)            # r_gripper_r_finger_joint

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


        if self.xml_path.find('baxter')>-1 and not self.xml_path.find('both')>-1:
            qpos[(self.all_joints*2):-gg] = 0
        elif self.xml_path.find('baxter')>-1:
            qpos[self.all_joints:-gg] = 0
        else:
            qpos[self.nn:-gg] = 0

        if self.xml_path.find('baxter')>-1:
            if self.xml_path.find('both')>-1:
                qpos[self.all_joints:-gg] = 0
            else:
                qpos[(self.all_joints*2):-gg] = 0
        else:
            qpos[self.nn:-gg] = 0
            self.goal[0] = np.random.uniform(-0.15,0.15)

        qpos[-gg:] = self.goal
        qvel = self.init_qvel
        self.set_state(qpos, qvel)

        if self.unity:
            self.remote.setqpos(self.sim.data.qpos)

        return self._get_obs()

    def get_robot_joints(self):
        if self.xml_path.find("leftarm")>-1:
            # print("baxter qpos",self.sim.data.qpos)
            # print("baxter left obs", self.sim.data.qpos.flat[self.all_joints:self.all_joints+self.nn])
            return np.concatenate([
                self.sim.data.qpos.flat[self.all_joints:self.all_joints+self.nn],
                self.sim.data.qvel.flat[self.all_joints:self.all_joints+self.nn]])
        return np.concatenate([
            self.sim.data.qpos.flat[:self.nn],
            self.sim.data.qvel.flat[:self.nn]])

    def get_finger_target(self):
        if self.xml_path.find("leftarm")>-1:
            return (self.sim.data.get_body_xpos("l_gripper_l_finger_tip") \
                + self.sim.data.get_body_xpos("l_gripper_r_finger_tip"))/2.0
        elif self.xml_path.find("rightarm")>-1 or self.xml_path.find("both")>-1:
            return (self.sim.data.get_body_xpos("r_gripper_l_finger_tip") \
                + self.sim.data.get_body_xpos("r_gripper_r_finger_tip"))/2.0
        else:
            assert "not sure about the end-effector type"

    def get_finger_ori(self):
        if self.xml_path.find("leftarm")>-1:
            return quat2euler(self.sim.data.get_body_xquat("left_wrist"))
        elif self.xml_path.find("rightarm")>-1 or self.xml_path.find("both")>-1:
            return quat2euler(self.sim.data.get_body_xquat("right_wrist"))
        else:
            assert "not sure about the end-effector type"
    
    def get_finger_quat(self):
        if self.xml_path.find("leftarm")>-1:
            return self.sim.data.get_body_xquat("left_wrist")
        elif self.xml_path.find("rightarm")>-1 or self.xml_path.find("both")>-1:
            return self.sim.data.get_body_xquat("right_wrist")  
        else:
            assert "not sure about the end-effector type"