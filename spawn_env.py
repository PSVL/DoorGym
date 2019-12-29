import gym
import doorenv
from random import randrange
from glfw import get_framebuffer_size
import numpy as np
import random

unity = False

env = gym.make('doorenv-v0', world_path='/home/demo/doorgym/world_generator/world/lever_baxter_rightarm')
if unity:
    env.env.init(0)
    env.env.change_model(env.env.xml_path)

viewer = env.env._get_viewer('human')
width, height = get_framebuffer_size(viewer.window)
env.env.viewer_setup(camera_type='global_cam', camera_select=0)

if env.env.xml_path.find("baxter")>-1:
    doorhinge_idx = 20
else:
    if env.env.xml_path.find("float")>-1:
        if env.env.xml_path.find("hook")>-1:
            doorhinge_idx = 6
        if env.env.xml_path.find("gripper")>-1:
            doorhinge_idx = 11
    else:
        if env.env.xml_path.find("hook")>-1:
            doorhinge_idx = 7
        if env.env.xml_path.find("gripper")>-1:
            doorhinge_idx = 12
    
env.env.reset()
env.env.sim.data.qpos[doorhinge_idx] = 0.3 #randrange(-100, 100)/100
env.env.set_state(env.env.sim.data.qpos, env.env.sim.data.qvel)
if unity:
    env.env.remote.setqpos(env.env.sim.data.qpos)
print(env.env.sim.data.qpos)

i = 0
while True:
    i += 1
    qpos = env.env.init_qpos
    qpos[0] = 0 # 0.08 #+ random.uniform(-1.70168, 1.70168)    # right_s0
    qpos[1] = 0 #-1.00 #+ random.uniform(-2.147, 1.047)        # right_s1
    qpos[2] = 0 # 1.19 #+ random.uniform(-3.05418, 3.05418)    # right_e0
    qpos[3] = 0 # 1.94 #+ random.uniform(-0.05, 2.618)         # right_e1
    qpos[4] = 0 #-0.67 #+ random.uniform(-3.059, 3.059)        # right_w0
    qpos[5] = 0 # 1.03 #+ random.uniform(-1.5708, 2.094)       # right_w1
    qpos[6] = 0 # 0.50 #+ random.uniform(-3.059, 3.059)        # right_w2
    qpos[7] = 0 # 0.02 #+ random.uniform(0, 0.020833)          # robotfinger_actuator_joint_r
    qpos[8] =  0.00 #+ random.uniform(-0.02, 0)            # r_gripper_l_finger_joint
    qpos[9] =  0.00 #+ random.uniform( 0, 0.02)            # r_gripper_r_finger_joint
    qpos[10] =  1.15 #+ random.uniform(-1.70168, 1.70168)    # left_s0
    qpos[11] =  1.05 #+ random.uniform(-2.147, 1.047)        # left_s1
    qpos[12] = -0.10 #+ random.uniform(-3.05418, 3.05418)   # left_e0
    qpos[13] =  0.50 #+ random.uniform(-0.05, 2.618)        # left_e1
    qpos[14] = -1.00 #+ random.uniform(-3.059, 3.059)       # left_w0
    qpos[15] = -0.01 #+ random.uniform(-1.5708, 2.094)      # left_w1
    qpos[16] = -1.92 #+ random.uniform(-3.059, 3.059)       # left_w2
    qpos[17] =  0.00 #+ random.uniform(0, 0.020833)         # robotfinger_actuator_joint_l
    qpos[18] =  0.00 #+ random.uniform(-0.02, 0)            # l_gripper_l_finger_joint
    qpos[19] =  0.00 #+ random.uniform( 0, 0.02)            # l_gripper_r_finger_joint

    env.env.set_state(qpos, env.env.sim.data.qvel)
    env.env.render(mode='human', width=width, height=height)

    if unity:
        env.env.remote.setqpos(env.env.sim.data.qpos)
        env.env.remote.setcamera(-1)
