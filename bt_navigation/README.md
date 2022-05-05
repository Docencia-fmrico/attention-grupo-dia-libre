ros2 launch br2_tiago sim.launch.py
rviz2 --ros-args -p use_sim_time:=true (OPCIONAL)
ros2 launch br2_navigation tiago_navigation.launch.py
ros2 launch bt_navigation nei_launcher.py
ros2 run bt_navigation nei_controller
