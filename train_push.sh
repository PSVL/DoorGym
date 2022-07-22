#! /bin/bash

python3 train.py --env-name doorenv-v0 --algo ppo --num-steps 4096 --num-processes 8 --lr 1e-3 --save-name husky_ur5_push_3dof --world-path ~/DoorGym/world_generator/husky-ur5-push/pull_husky_ur5/