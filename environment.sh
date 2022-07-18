#! /bin/bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/kl/.mujoco/mujoco210/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia
export PYTHONPATH=$PYTHONPATH:/home/kl/DoorGym/DoorGym-Unity/python_interface
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so:/usr/lib/x86_64-linux-gnu/libEGL.so