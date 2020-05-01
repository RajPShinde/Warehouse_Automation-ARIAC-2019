# group2_rwa5
Build Instructions...

Extract the group2_rwa5.zip inside "~/catkin_ws/src/"
->open a terminal and the run following command in the terminal.
```
cd ~/catkin_ws/
catkin_make --only-pkg-with-deps group2_rwa5
```

Run Instructions
...
1. Open terminal
2. Type following commands in the terminal
 ```Terminal 1:
source ~/catkin_ws/devel/setup.bash
roslaunch group2_rwa5 group2-rwa5.launch
 ```
3. Run following commands in new terminals:
 ```Terminal 2:
source ~/catkin_ws/devel/setup.bash
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1
```Terminal 3:
source ~/catkin_ws/devel/setup.bash
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm2
'''Terminal 4:
source ~/catkin_ws/devel/setup.bash
rosrun group2_rwa5 main_node
