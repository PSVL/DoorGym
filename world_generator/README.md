# How to use

## Step0: Download the doorknob data set
#### [Pull knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/pullknobs.tar.gz) (0.75 GB)
#### [Lever knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/leverknobs.tar.gz) (0.77 GB)
#### [Round knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/roundknobs.tar.gz) (1.24 GB)

## Step1: Configure the folder structure as follow
* Extract and place the downloaded door knob dataset under the `door` folder (or make symlink).
* Place your favorite robots under the `robot` folder. (Blue robots are there as default)
* `world` folder will be the output folder

```bash
world_generator
├── world_genterator.py
├── mjcf
├── door
|   ├── pullknobs
|   ├── roundknobs
│   └── leverknobs
│       ├── knobfolder1
│       |   ├── body_1.stl
│       |   ├── body_2.stl
│       |   |      |  
│       |   ├── body_8.stl
│       |   ├── info.json
│       |   └── preview.png
│       |          |
│       └── knobfolder100
│           ├── body_1.stl
│           ├── body_2.stl
│           |      |  
│           ├── body_8.stl
│           ├── info.json
│           └── preview.png
├── robot
│   ├── blue_hook.xml
│   ├── blue_gripper.xml
│   ├── blue_floatinghook.xml
│   ├── blue_floatinggripper.xml
│   ├── blue_mobilehook.xml
│   ├── blue_mobilegripper.xml
│   └── STL folder
│       ├── base_link.STL
│       ├── shoulder_link.STL
│       └── arm_link.STL
└── world
    ├── knobfolder0_round_blue_hook.xml
    ├── knobfolder1_lever_blue_hook.xml
    ├──             |    
    └── knobfolder100_pull_blue_floatingwrist.xml
```


## Step2: Run following (e.g. lever knob and hook arm combination.)
`python3 world_generator.py --knob_type lever --robot_type blue_hook`

## Step3: Check the model by running the mujoco simulator
`cd ~/.mujoco/mjpro150/bin`
`./simulate ../path/to/DoorGym/world_generator/world/knobfolder0_lever_blue_hook.xml`

# Reference
This mjcf generator is powered by the following repo.

[https://github.com/iandanforth/mjcf](https://github.com/iandanforth/mjcf)
