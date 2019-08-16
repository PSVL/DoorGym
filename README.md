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
### 8/13/2019 Random Door knob Generator has been released!
### 8/12/2019 Door knob Dataset (3K knobs for each type) has been released!

## 0. Set up the environment
requirement:

-Python3.6 or later

-Mujoco

-Mujoco-py

-Gym

-OpenAI Baseline

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

## DoorGym is now on it's way! (ETA Aug 25 2019)

Door Gym will includes follows.
- [x] Door knob data set (Pull knob, Lever knob, Round knob 3K each)
- [x] Random Door world generator
- [ ] Mujoco-Unity Plugin
- [ ] Door knob position vision net trainer
- [ ] Door opening policy trainer

## Paper
https://arxiv.org/abs/1908.01887
