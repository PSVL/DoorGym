## Basic steps to make door worlds.

## Step0: Download the doorknob data set
#### [Pull knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/pullknobs.tar.gz) (0.75 GB)
#### [Lever knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/leverknobs.tar.gz) (0.77 GB)
#### [Round knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/roundknobs.tar.gz) (1.24 GB)

## Step1: Configure the folder structure.
* Extract and place the downloaded door knob dataset under the `door` folder (or make a symlink).
* Place your favorite robots under the `robot` folder. (Blue robots are there as default)

## Step2: Run following (e.g. lever knob and hook arm combination.)
`cd path/to/DoorGym/`

`python3 ./world_generator/world_generator.py --knob_type lever --robot_type floatinghook`

## Step3: Check the model by running the mujoco simulator
`cd ~/.mujoco/mjpro150/bin`

`./simulate path/to/DoorGym/world_generator/world/lever_floatinghook/1551848929_lever_floatinghook.xml`

## Arguments

| name          | options       | default   | explaination   |
| ------------- |-------------| --------| -------------|
| knob-type     | "lever", "round" or "pull" | ' ' |If no arg, it use all types.|
| robot-type    | "gripper","hook","floatinghook","floatinggripper", "mobile_gripper" or "mobile_hook"   | 'floatinghook' | - |
| one-dir       | True/False      | False | Save everything into one dir, or save into separate dir by its robot and knob types|
| pulldoor-ratio       | 0.0~1.0  |   1.0 | ratio of door that opens by pulling. |
| righthinge-ratio     | 0.0~1.0  |   1.0 | ratio of door that has hinge on right side. |

# Reference
This mjcf generator is powered by the following repo.

[https://github.com/iandanforth/mjcf](https://github.com/iandanforth/mjcf)
