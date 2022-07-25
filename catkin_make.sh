#! /bin/bash

source install.sh

# all
catkin_make -C ./catkin_ws

source catkin_ws/devel/setup.bash