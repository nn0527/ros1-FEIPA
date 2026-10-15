# Workspace rules

## Build after program changes

- After modifying any program in this ros_ws workspace, compilation MUST be performed inside Docker.
- Use the existing ros1_noetic container. The host /home/pi/ros_ws is mounted at /root/ros_ws in the container.
- Do not compile this workspace directly on the Raspberry Pi host.
- Run the following command after program changes:

```bash
docker exec ros1_noetic bash -lc 'source /opt/ros/noetic/setup.bash && cd /root/ros_ws && catkin_make'
```

- If the container is unavailable or compilation fails, report the blocker; do not claim build verification succeeded or substitute a host build.
