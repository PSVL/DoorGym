# How to use. (Not ready yet. Please wait) 

## Step0: Download the doorknob data set
#### [Pull knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/pullknobs.tar.gz) (0.75 GB)
#### [Lever knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/leverknobs.tar.gz) (0.77 GB)
#### [Round knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/roundknobs.tar.gz) (1.24 GB)

## Step1: Configure the folder structure as follow
* Extract and place the downloaded door knob dataset under the `door` folder (or make symlink).
* Place your favorite robots under the `robot` folder. (Blue robots are there as default)
* `world` folder will be the output folder

```bash
doorgym
└──  world_generator
    ├── world_genterator.py
    ├── mjcf
    ├── door
    |   ├── pullknobs
    |   ├── roundknobs
    │   └── leverknobs
    │       ├── 1551848929
    │       |   ├── body_1.stl
    │       |   ├── body_2.stl
    │       |   |      |  
    │       |   ├── body_8.stl
    │       |   ├── info.json
    │       |   └── preview.png
    │       |          |
    │       └── 1551849078
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
        ├── world
        └── lever_floatinghook
            ├── 1551848929_lever_floatinghook.xml
            ├── 1551848946_lever_floatinghook.xml
            ├──             |    
            └── 1551849078_lever_floatinghook.xml
```


## Step2: Run following (e.g. lever knob and hook arm combination.)
`cd path/to/DoorGym/`

`python3 ./world_generator/world_generator.py --knob_type lever --robot_type floatinghook`

## Step3: Check the model by running the mujoco simulator
`cd ~/.mujoco/mjpro150/bin`

`./simulate path/to/DoorGym/world_generator/world/lever_floatinghook/1551848929_lever_floatinghook.xml`

# Reference
This mjcf generator is powered by the following repo.

[https://github.com/iandanforth/mjcf](https://github.com/iandanforth/mjcf)
