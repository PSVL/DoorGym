# DoorGym

## Clone repo and Docker

I released two version docker images, one is for developers that can modify other packages on this and compile by yourself, the other one is for user that easy to use without extra install other related libraries. 

### Step1. Clone repo

```
  git clone --recursive git@github.com:kuolunwang/DoorGym.git
```

### Step2. Docker run

Run this script to pull the docker image to your workstation.
* For the development mode

  ```
  source docker_run.sh
  ```

* For the user mode
  ```
  source docker_run_user.sh
  ```

### Step3. Docker join

If you want to enter the same docker image, type below command.

* For the development mode
  ```
  source docker_join.sh
  ```

* For the user mode

  ```
  source docker_join_user.sh
  ```

### Step4. Catkin_make

Execute the compile script at first time, then the other can ignore this step. 

```
cd catkin_ws && catkin_make
```

### Step5. Setup environment

Make sure run this command when the terminal enter docker, for the user mode you can ignore install.sh.

```
source environment.sh
```

* For the development mode

  ```
  source install.sh
  ```

## Download dataset and knob model

This dataset is required for both training and evaluation, you can follow below command to download these. 

```
cd world_generator
python3 download_data.py
```

If you want to make your own dataset, you can follow below steps.

### Step1. Lever and Round knobs

#### [Lever knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/leverknobs.tar.gz) (0.77 GB)
#### [Round knobs](https://github.com/PSVL/DoorGym/releases/download/v1.0/roundknobs.tar.gz) (1.24 GB)

### Step2. Setup knob and robot model

* Extract the downloaded door knob dataset under the `world_generator/door` folder.
* Place your favorite robots under the `world_generator/robot` folder. (Blue robots are there as default)

### Step3. Generate door world (e.g. pull knob and husky ur5 combination.)

```
cd ~/DoorGym/world_generator
python3 world_generator.py --knob-type pull --robot-type husky-ur5
```

Check the model by running the mujoco simulator

```
cd ~/.mujoco/mujoco210/bin
./simulate [environment path]
```

More detailed instruction [here](./world_generator)

## Training and evaluation

Before ready for the above setting, then you can train a model or evaluate a pre-trained model. For the training and evaluation are need parameter setting, I arrange it and write a script that is easy to use.

### Training

* Pull task

  ```
    source train_pull.sh
  ```

* push task

  ```
    source train_push.sh
  ```

### Evaluation

* Pull task

  ```
    source run_pull.sh
  ```

* push task

  ```
    source run_push.sh
  ```

More detailed instruction [here](https://github.com/ARG-NCTU/curl_navi/blob/master/04_DoorGym.ipynb)