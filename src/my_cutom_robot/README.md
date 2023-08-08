## Tips:
''''
ros2 run joint_state_publisher_gui joint_state_publisher_gui 
''''
let your joint visible.

''''
ros2 launch gazebo_ros gazebo.launch.py
''''
Open your gazebo

''''
ros2 launch my_cutom_robot rsp.launch.py use_sim_time:=true
''''
Sync your rviz2 with Gazebo 

''''
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity demo
''''
Spawn your urdf robot into gazebo 


## Due to the complexity of publishing different nodes once
Use:
    launch_sim.launch.py