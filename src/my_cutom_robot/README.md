## Tips:

在仿真中使joint可见，运行以下命令：

    ros2 run joint_state_publisher_gui joint_state_publisher_gui 

let your joint visible.

运行Gazebo仿真

    ros2 launch gazebo_ros gazebo.launch.py

Open your gazebo

打开sim_time:=true, 使得rivz2 和 Gazobo 乃至未来的 foxglove在时间戳上进行话题信息上的同步，以便仿真使用 

    ros2 launch my_cutom_robot rsp.launch.py use_sim_time:=true

Sync your rviz2 with Gazebo 

将话题 robot_description 中的robot model “放”到Gazobo仿真中， 举一个很简单的例子，使在 rviz2 那些带颜色的 urdf在 gazebo中也能带颜色（这个比较肤浅，当然还有使机器人在仿真中更真实的物理特性，比如静态摩擦力参数等）

    ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity demo
Spawn your urdf robot into gazebo 


    ros2 run teleop_twist_keyboard teleop_twist_keyboard 
Move your robot in Gazebo


    sudo apt install ros-humble-image-transport-plugis
    sudo apt install ros-humble-rqt-image-view
    ros2 run rqt_image_view rqt_image_view 

To compress your image when transport them in different topics

## Due to the complexity of publishing different nodes once
Use:
    launch_sim.launch.py
