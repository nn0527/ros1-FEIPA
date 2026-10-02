& wsl.exe -d Ubuntu-20.04 -- bash --noprofile --norc -c 'source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11351 ROS_HOSTNAME=127.0.0.1; rosnode kill /sensor_range_manager_node'
exit $LASTEXITCODE
