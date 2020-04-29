# ARIAC-2019-Final
# group2_final

Group members:
1. Chinmay Joshi
2. Rachith Prakash
3. Raj Shinde
4. Shesh Mali
5. Shubham Sonawane

Build Instructions...

Extract the group2_final.zip inside "~/catkin_ws/src/"
->open a terminal and the run following command in the terminal.
```
cd ~/catkin_ws/
catkin_make --only-pkg-with-deps group2_final
```

Run Instructions
...
1. Open terminal
2. Type following commands in the terminal
 ```Terminal 1:
source ~/catkin_ws/devel/setup.bash
roslaunch group2_final group2-final.launch
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
rosrun group2_final main_node
```

# Note: 
We are able to fulfill the order from the parts present in the bins. We had prepared our system for picking from conveyor as well, but due to shortage of time, we couldnt perfect it. 
