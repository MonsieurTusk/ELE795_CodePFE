#!/bin/bash
xterm -e "source /opt/ros/humble/setup.bash; ros2 bag play bag_files/bag_lidar_fixe/ --loop" &
sleep 1
xterm -e "source /opt/ros/humble/setup.bash; colcon build; source install/setup.bash; ros2 run talker_listener talkerNode_m1m2" &
sleep 10
xterm -e "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run talker_listener listenerNode" & 
sleep 1 
xterm -e "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run talker_listener rapportNode" & 
sleep 1
xterm -e "rviz2 -d talker_listener/PFE_PointCloud2.rviz"
sleep 1
