# How to use

## Step0: Download the doorknob data set

## Step1: Configure the folder structure as follow
* Place the downloaded door knob dataset under the `door` folder.
* Place your favorite robots under the `robot` folder. (Blue robots as default)
* `world` folder will be the output folder

```bash
Panasonic_DoorWorldGenerator
├── world_genterator.py
├── mjcf
├── door
|   ├── 3000_pullknobs
|   ├── 3000_roundknobs
│   └── 3000_leverknobs
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
│   ├── blue_full.xml
│   ├── blue_hook.xml
|   |          |
│   ├── blue_floatingwrist.xml
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


## Step2: Run following
`python3 world_generator.py --knob_type lever --robot_type blue_hook`

## Step3: Check the model by running the mujoco simulator
`cd ~/.mujoco/mujoco_pro200/bin`
`./simulate ..../Panasonic_DoorWorldGenerator/world/knobfolder0_lever_blue_hook.xml`

# Reference
This mjcf generator is powered by the following repo.

[https://github.com/iandanforth/mjcf](https://github.com/iandanforth/mjcf)