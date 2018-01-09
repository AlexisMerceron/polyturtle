#!/bin/bash

if [ ! -z $1 ]; then
    IP_TURTLEBOT=$1
else
    IP_TURTLEBOT="192.168.43.42"
fi

echo "Starting roscore, please wait while loading"
xterm -geometry 80x24+0+0 -e "roscore" &
roscore_pid=$!
read -n1 -r -p "Press key to continue..." key

echo -e "\n\nBringing up Turtlebot, please wait while loading"
xterm -geometry 80x24+0-0 -e "ssh pi@$IP_TURTLEBOT bash -ic bringup" &
bringup_pid=$!
read -n1 -r -p "Press key to continue..." key

echo -e "\n\nStarting SLAM, please wait while loading"
xterm -geometry 80x24-0-0 -e "roslaunch turtlebot3_slam turtlebot3_slam.launch" &
slam_pid=$!
read -n1 -r -p "Press key to continue..." key

echo -e "\n\nLoading RViz, please wait while loading"
xterm -geometry 80x24-0-0 -e "rosrun rviz rviz -d `rospack find turtlebot3_navigation`/rviz/turtlebot3_nav.rviz" &
rvizslam_pid=$!
read -n1 -r -p "Press key to continue..." key

echo -e "\n\nStarting teleoperation, please wait while loading"
xterm -geometry 80x24-0+0 -e "roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch" &
teleop_pid=$!

echo -e "\n"
read -n1 -r -p "Press key to STOP !" key

kill $teleop_pid
kill $bringup_pid
ssh pi@$IP_TURTLEBOT "pid=\$(ps aux | grep '[r]oslaunch' | awk '{print \$2}' | head -1); echo \$pid | xargs kill"
kill $slam_pid
kill $rvizslam_pid
kill $roscore_pid

echo -e "\n"
