#!/bin/bash

echo "starting next phases"

source ~/catkin_ws/devel/setup.bash

rosnode kill /turtlebot3_slam_gmapping

sleep 1 

roslaunch final_rbe3002 3rd_phase.launch

echo "next phase started"
