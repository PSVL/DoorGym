# DoorGym

[doorgym_video]: ./imgs/doorgym_video.gif

<!---
<p align="center">
  ![alt text][doorgym_video]
</p>
--->

<p align="center">
  <img width="460" height="406" src="./imgs/doorgym_video.gif">
</p>

## Updates
#### 8/30/2019 Doorgym-Unity plugin has been released!
#### 8/26/2019 Door opening agent trainer has been released!
#### 8/13/2019 Random Door knob Generator has been released!
#### 8/12/2019 Door knob Dataset (3K knobs for each type) has been released!

DoorGym includes follows.
- [x] Door knob data set (Pull knob, Lever knob, Round knob 3K each)
- [x] Random Door world generator
- [x] Door opening policy trainer
- [x] Mujoco-Unity Plugin ver. Doorgym

## 0. Set up the environment
requirement:

-Python3.6 or later

-Mujoco

-Mujoco-py

-Gym 0.14.0 or later

-OpenAI Baseline

-Unity3D

etc.

We are currently working on making requirement.txt and envronment package (Anaconda or Docker).

## 1. Download the randomized door knob dataset
You can download from the following URL (All tar.gz file).
#### [Pull knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/pullknobs.tar.gz) (0.75 GB)
#### [Lever knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/leverknobs.tar.gz) (0.77 GB)
#### [Round knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/roundknobs.tar.gz) (1.24 GB)

## 2. Generate door world (e.g. lever knob and hook arm combination.)
`cd path/to/DoorGym/`

`python3 ./world_generator/world_generator.py --knob_type lever --robot_type floatinghook`

Check the model by running the mujoco simulator

`cd ~/.mujoco/mjpro150/bin`

`./simulate path/to/DoorGym/world_generator/world/lever_floatinghook/1551848929_lever_floatinghook.xml`

More detailed instruction [here](./world_generator)

Register the doorenv as the gym environment.
`cd DoorGym/envs`

`pip install -e .`

## 3. train the agent on the generated door worlds (e.g. lever knob and hook arm combination.)
### Proximal Policy Optimization (PPO) training
`python main.py --env-name doorenv-v0 --algo ppo --num-steps 4096 --num-processes 8 --lr 1e-3 --save-name ppo-test`

### Soft Actor Critic (SAC) training
`python main.py --env-name doorenv-v0 --algo sac --save-name ppo-test`

### Twin Delayed DDPG (TD3) and Advantage Actor-Critic (A2C) can be used as following arguments
`--algo td3` or `--algo a2c`.

## 4. Train with vision network estimator
### with Unity
`python main.py --env-name doorenv-v0 --algo ppo --num-steps 4096 --num-processes 8 --lr 1e-3 --save-name ppo-test --visionnet-input --unity`
### without Unity
`python main.py --env-name doorenv-v0 --algo ppo --num-steps 4096 --num-processes 8 --lr 1e-3 --save-name ppo-test --visionnet-input`

Download and configure DoorGym-Unity Plugin from [here](https://github.com/PSVL/DoorGym-Unity).

## 5. Run the policy
`python enjoy.py --env-name doorenv-v0 --load-name trained_models/ppo/doorenv-v0_reacher-pull-floatinghook.600.pt`

## Paper
https://arxiv.org/abs/1908.01887

## Reference
This repo is powered by following excellent repos and software.
- [ikostrikov/pytorch-a2c-ppo-acktr-gail](https://github.com/ikostrikov/pytorch-a2c-ppo-acktr-gail)
- [vitchyr/rlkit](https://github.com/vitchyr/rlkit)
- [iandanforth/mjcf](https://github.com/iandanforth/mjcf)
- [Mujoco-Unity Plugin](http://www.mujoco.org/book/unity.html)

