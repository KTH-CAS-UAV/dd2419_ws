**Readme about the detection part of the detection part of Milestone 1:**

For playing back the rosbag run the following nodes /launch files:

pixi run rviz2
pixi run ros2 run odometry odometry  #important to have transform from base_link to odom
pixi run ros2 launch robp_launch frames_launch.xml
pixi run ros2 bag play --read-ahead-queue-size 100 -l -r 1.0 --clock 100 --start-paused ~/dd2419_ws/rosbags/obj_det_odom_opti_rosbag
pixi run ros2 run detection detection

