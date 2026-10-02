#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/wallrobot/catkin_ws_original/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11351
export ROS_HOSTNAME=127.0.0.1
fcu_port="${1:?Specify FCU serial device}"
baud="${2:-57600}"
ceiling_port="${3:-}"
front_port="${4:-}"
[[ "$ceiling_port" != '-' ]] || ceiling_port=''
[[ "$front_port" != '-' ]] || front_port=''
if rosparam list >/dev/null 2>&1; then
  echo 'ROS master on 11351 already running; stop the current observation first.' >&2
  exit 2
fi
for device in "$fcu_port" "$ceiling_port" "$front_port"; do
  [[ -z "$device" ]] && continue
  [[ -c "$device" && -r "$device" && -w "$device" ]] || { echo "Serial device unavailable or not accessible: $device" >&2; exit 2; }
done
[[ "$fcu_port" != "$ceiling_port" && "$fcu_port" != "$front_port" ]] || { echo 'FCU and sensors must use different ports' >&2; exit 2; }
[[ -z "$ceiling_port" || "$ceiling_port" != "$front_port" ]] || { echo 'Two sensors must use different ports' >&2; exit 2; }
runtime_dir="$(cd "$(dirname "$0")" && pwd)"
mkdir -p /home/wallrobot/catkin_ws_original/logs
roslaunch -p 11351 "$runtime_dir/usb_observe.launch" fcu_url:="$fcu_port:$baud" ceiling_serial_port:="$ceiling_port" front_serial_port:="$front_port" 2>&1 | tee "/home/wallrobot/catkin_ws_original/logs/observe_$(date +%Y%m%d_%H%M%S).log"
