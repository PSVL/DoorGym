# DoorGym

[doorgym_video]: ./imgs/doorgym_video.gif

<!---
<p align="center">
  ![alt text][doorgym_video]
</p>
--->
<p align="center">
  <img width="345" height="304" src="./imgs/doorgym_video.gif">
  <img width="345" height="304" src="./imgs/baxter.gif">
</p>

## Updates
#### 12/14/2019 Presenting poster at [Deep RL workshop at NeurIPS 2019](https://sites.google.com/view/deep-rl-workshop-neurips-2019/home)! ([paper](https://drive.google.com/file/d/1rC9wpd-4AMBgDMq8skAQIdTER8wSARMs/view))
#### 11/3/2019 Now you can choose Baxter as a robot!
#### 8/30/2019 [Doorgym-Unity plugin](https://github.com/PSVL/DoorGym-Unity) has been released!
#### 8/26/2019 Door opening agent trainer has been released!
#### 8/13/2019 Random Door knob Generator has been released!
#### 8/12/2019 Door knob Dataset (3K knobs for each type) has been released!

DoorGym includes follows.
- [x] Door knob data set (Pull knob, Lever knob, Round knob 3K each)
- [x] Robot xml (UC Berkeley's BLUE, Rethinkrobot's Baxter)
- [x] Random Door world generator
- [x] Door opening policy trainer
- [x] Mujoco-Unity Plugin ver. Doorgym

## 0. Set up the environment
requirement:

- Ubuntu 16.04 <

-Python3.6 or later

-Mujoco

-Mujoco-py

-Gym 0.14.0 or later

-OpenAI Baseline 0.1.6 or later

-Unity3D

etc.

### Conda (Anaconda, Miniconda)
#### Step1. Install Mujoco
1. Get the license from [MuJoCo License page](https://www.roboti.us/license.html)

2. Download Mujoco2.00 from [MuJoCo Product page](https://www.roboti.us/index.html).

3. Extruct it and place it in under `home/.mujoco`, as `/mujoco200` (Not mujoco200_linux).

4. Put your key under both `.mujoco/` and `.mujoco/mujoco200/bin`.

Detailed installation can be checked in following page.
https://github.com/openai/mujoco-py

#### Step2. Set environment var. and install necessary pkgs
```bash
sudo apt install libosmesa6-dev libgl1-mesa-glx libglfw3 libglew-dev libopenmpi-dev patchelf
```
Set following path in `~/.bashrc`.
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/[usr-name]/.mujoco/mujoco200/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia-[driver-ver]
export PYTHONPATH=$PYTHONPATH:/home/[use-name]/DoorGym/DoorGym-Unity/python_interface
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so
```

#### Step3. Clone DoorGym repo and Create conda environment
```bash
git clone https://github.com/PSVL/DoorGym
cd ./DoorGym
git submodule init
git submodule update
conda env create -n doorgym -f environment/environment.yml
conda activate doorgym
pip install -r requirements.txt
pip install -r requirements.dev.txt
```

#### Step4. Install doorenv (0.0.1)
```bash
cd envs
pip install -e .
```

#### Step5. Install OpenAI Baselines (0.1.6<)
```bash
cd ../..
git clone https://github.com/openai/baselines
cd baselines
pip install -e .
```

#### Step6. Import test
Make sure that you can import pytorch and mujoco-py w/o problem.
```bash
Python 3.7.4 (default, Aug 13 2019, 20:35:49) 
[GCC 7.3.0] :: Anaconda, Inc. on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import torch
>>> torch.cuda.is_available()
True
>>> import mujoco_py
```

If there is an error while importing mujoco-py, try trouble shooting according to the [mujoco-py installation guide](https://github.com/openai/mujoco-py)

## 1. Download the randomized door knob dataset
You can download from the following URL (All tar.gz file).
#### [Pull knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/pullknobs.tar.gz) (0.75 GB)
#### [Lever knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/leverknobs.tar.gz) (0.77 GB)
#### [Round knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/roundknobs.tar.gz) (1.24 GB)

* Extract and place the downloaded door knob dataset under the `world_generator/door` folder (or make a symlink).
* Place your favorite robots under the `world_generator/robot` folder. (Blue robots are there as default)

## 2. Generate door world (e.g. lever knob and hook arm combination.)
`cd [path/to/]DoorGym/world_generator`

`python world_generator.py --knob-type lever --robot-type floatinghook`

Check the model by running the mujoco simulator

`cd ~/.mujoco/mujoco200/bin`

`./simulate [path/to/DoorGym]/DoorGym/world_generator/world/lever_floatinghook/1551848929_lever_floatinghook.xml`

More detailed instruction [here](./world_generator)

## 3. train the agent on the generated door worlds (e.g. lever knob and hook arm combination.)
### Proximal Policy Optimization (PPO) training

`python main.py --env-name doorenv-v0 --algo ppo --num-steps 4096 --num-processes 8 --lr 1e-3 --save-name ppo-test --world-path /[abs_path/to/DoorGym/]DoorGym/world_generator/world/pull_floatinghook`

### Soft Actor Critic (SAC) training
`python main.py --env-name doorenv-v0 --algo sac --save-name ppo-test --world-path /[abs_path/to/DoorGym/]DoorGym/world_generator/world/pull_floatinghook`

### Twin Delayed DDPG (TD3) and Advantage Actor-Critic (A2C) can be used as following arguments
`--algo td3` or `--algo a2c`.

## 4. Train with vision network estimator
Install [Unity3D](https://forum.unity.com) editor for Linux.

Open the project in DoorGym-Unity Plugin from submodule `DoorGym-Unity`.
### with Unity
`python main.py --env-name doorenv-v0 --algo ppo --num-steps 4096 --num-processes 8 --lr 1e-3 --save-name ppo-test --world-path /[abs_path/to/DoorGym/]DoorGym/world_generator/world/pull_floatinghook --visionnet-input --unity`
### without Unity
`python main.py --env-name doorenv-v0 --algo ppo --num-steps 4096 --num-processes 8 --lr 1e-3 --save-name ppo-test --world-path /[abs_path/to/DoorGym/]DoorGym/world_generator/world/pull_floatinghook --visionnet-input`

## 5. Run the policy
`python enjoy.py --env-name doorenv-v0 --load-name trained_models/ppo/doorenv-v0_reacher-pull-floatinghook.600.pt --world-path /[abs_path/to/DoorGym/]DoorGym/world_generator/world/pull_floatinghook`

## Paper
Newer paper with Sim2Real ([NeurIPS workshop](https://drive.google.com/file/d/1rC9wpd-4AMBgDMq8skAQIdTER8wSARMs/view))

Arxiv paper: https://arxiv.org/abs/1908.01887

## Reference
This repo is powered by following excellent repos and software.
- [ikostrikov/pytorch-a2c-ppo-acktr-gail](https://github.com/ikostrikov/pytorch-a2c-ppo-acktr-gail)
- [vitchyr/rlkit](https://github.com/vitchyr/rlkit)
- [iandanforth/mjcf](https://github.com/iandanforth/mjcf)
- [Mujoco-Unity Plugin](http://www.mujoco.org/book/unity.html)

