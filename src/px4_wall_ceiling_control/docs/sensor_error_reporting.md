# 两路雷达的失效原因

原始距离输入：`/sensor/front/raw`、`/sensor/ceiling/raw`，类型 `sensor_msgs/Range`，`range` 单位为米。校验后的输出：`/sensor/front/range`、`/sensor/ceiling/range`。

在 `/sensor/health`（`px4_wall_ceiling_control/SensorHealth`）中，顶部仍使用 `ceiling` 命名：

```yaml
front_valid: false
front_error_code: 255
front_error_type: sensor
front_error_reason: "L1 error 255 (weak reflection or calculation failure)"
```

顶部使用 `ceiling_valid`、`ceiling_error_code`、`ceiling_error_type`、`ceiling_error_reason`。有效数据恢复时错误码清零，原因清空。两路独立处理。

|错误码|类型|含义|
|---|---|---|
|0|空|有效|
|正数，如 255、258|sensor|传感器返回的原始 L1 错误码；未知码也保留|
|-1|timeout|数据超时|
|-2|no_data|节点启动后尚未收到距离数据|
|-3|nonfinite|距离为 NaN 或无穷大|
|-4|out_of_range|距离超出配置量程|
|-5|parse|串口帧无法解析|
|-6|configuration|串口未配置|
|-7|driver|缺少串口驱动依赖|
|-8|serial|串口打开或读写异常|
|-9|diagnostic|诊断消息格式错误|

驱动详细诊断：`/sensor/front/diagnostic`、`/sensor/ceiling/diagnostic`，类型 `std_msgs/String`，JSON 包含 `valid`、`error_code`、`error_type`、`error`、`stamp`、`raw_frame`。例如 `raw_frame` 为 `E=255`。超时不伪装为传感器原生错误码。

容器中查看：

```bash
source /root/ros_ws/devel/setup.bash
rostopic echo /sensor/health
rostopic echo /sensor/front/diagnostic
rostopic echo /sensor/ceiling/diagnostic
```

`SensorHealth.msg` 增加了字段，ROS 消息 MD5 会改变。编译后应在停桨状态重新启动整套基础 ROS 程序，让所有发布者及订阅者加载同一消息版本；仅重启一个节点可能无法连接旧消息版本。现有录制器记录 `/sensor/health`，因此新错误字段会保存在后续 rosbag 中。Range 原始话题不是磁盘目录。
