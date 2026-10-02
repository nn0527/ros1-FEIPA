& wsl.exe -d Ubuntu-20.04 -- bash --noprofile --norc -c 'source /opt/ros/noetic/setup.bash; source /home/wallrobot/catkin_ws_original/devel/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11351 ROS_HOSTNAME=127.0.0.1; python3 /mnt/c/Users/60497/Desktop/catkin_ws/runtime/telemetry_view.py'
exit $LASTEXITCODE
