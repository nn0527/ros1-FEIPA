# 自动测试日志：照常启动，测试后交目录

记录范围仅为本项目ROS程序及其可接收的信号，不收集机身参数，不要求填写测试模板，不优化飞行策略。

## 操作者需要做什么

1. 同步项目后按原方式构建、加载环境；这一步只需在更新代码后做。
2. 照常启动`attachment_terminal_flight.launch`或`attachment_system_mavros.launch`，照常操作。记录默认随系统启动，无需另开录制终端。
3. 测试后正常Ctrl+C停止原launch，等待退出；将本次`~/wall_test_records/时间_编号/`整个目录交给分析者，并口述当时现象。

无需人工事件标记、元数据填写或离线分析命令。每次启动自动生成独立目录，名称使用UTC时间；终端会提示`Automatic flight logs: ...`。

启动方式和飞行参数不变。仅增加一个独立、非required的旁路记录节点，异常不会因required设置连带关闭控制节点。终端操作节点仍输出到屏幕；其他控制/传感器节点的stdout/stderr改由roslaunch保存到文件，便于保留未捕获异常的堆栈。

## 自动保存什么

| 文件 | 内容与分析作用 |
|---|---|
| `events.log` | 状态/故障原因、PX4模式、owner、飞行阶段、ROS警告/错误；用于先对照口述确定异常时间 |
| `signals.csv` | 约10 Hz可读信号快照：实测/目标姿态、角速度、位置/竖直速度、推力、测距、数据年龄、估计器状态；用于检查目标与实际差异 |
| `flight_*.bag` | 原始话题消息，保留原发布频率；用于精确重建消息顺序、波形和异常前后的控制链路 |
| `ros_logs/` | 每2秒复制一次原roslaunch日志，包含配置为log输出的节点stdout/stderr和异常堆栈 |
| `ros_parameters.json` | 启动时实际ROS参数，避免把默认配置误当成此次运行值 |
| `package_snapshot.zip`、`source_sha256.json` | 当前可找到的包代码/配置/消息/launch与校验值，帮助确认运行版本 |
| `session.json` | 记录时间、接收消息数、录制结果、原roslaunch日志位置与记录工具自身异常 |
| `recorder.log` | rosbag订阅与缓冲等信息，用于检查录制链路本身 |

话题覆盖状态/候选/最终姿态推力、FCU目标回传、odom/IMU/估计器、雷达原始与处理值、操作与脱离指令、RC、电池、GPS/高度相关输出、ROS/PX4状态文本。不存在或未发布的话题不会产生数据；接收计数可帮助识别缺项。

订阅和录制启动不构成飞行准入检查。若要确认记录已工作，只需查看终端有无记录错误；首次部署仍需在ROS机器做一次短时启动/停止，确认生成CSV、事件日志和bag。当前Windows工作区不能替代真实ROS录制验证。

## 程序崩溃和轨迹异常如何回溯

- 控制节点崩溃：独立rosbag继续记录其他仍发布的信号，文件日志保留Python异常堆栈；分析时查看该节点最后输出、状态和故障上下文。
- 轨迹异常：对齐侧吸状态、控制候选、最终发送目标与实测姿态，检查误差、角速度、推力、高度和测距有效性如何变化。
- 整个launch正常停止：记录节点给自己的rosbag进程发送SIGINT，关闭当前bag并保存结束信息。
- 记录节点自身异常：记录错误会提示并写入会话信息；记录失败不等于控制失败，不影响现有控制算法。
- 强制杀进程或断电：尽量保留之前已经关闭的分段bag；正在写入的`.bag.active`可能需要恢复索引，末尾缓冲数据可能丢失。不能承诺断电时零丢失，分析者会先核对完整性。

bag按30秒ROS记录时长分段，已有数据不会自动清理或覆盖。磁盘不足1 GB时停止记录并报告错误。记录与日志复制会消耗磁盘/CPU；正式测试前的短时验证应确认没有明显负载影响。

## 分析时必须遵守的解释规则

- CSV是定期读取最近收到的数据，不是所有传感器硬件同步采样。保留UTC、ROS时间、源header时间和各数据接收年龄；不要忽略陈旧数据。
- 目标与实测误差用四元数最小旋转夹角。目标超过0.25 s、odom超过0.5 s，或目标掩码忽略姿态时不输出误差；空值不代表0。
- 三条目标路径区分决策候选、ROS最终发送、FCU目标回传；FCU目标回传不等于实测姿态。
- CSV竖直速度按记录的`odom_twist_in_child_frame`解释，机体系速度转到世界系；原始odom仍保留在bag。
- 状态机退出不等于物理脱墙成功；估计位置/高度变化也不必然是真实运动。结合口述与信号一致性判断。
- ROS侧收到的RC输出不是实际RPM或完整电机饱和诊断。这套日志足以分析ROS控制链路及已收到的信号；若问题发生在PX4内部且ROS没有对应数据，再补充ULog，不凭空推断。
- 原roslaunch日志目录也记录在`session.json`。正常结束时最后复制可能与其他节点退出并发；若某个末尾堆栈未复制，分析者可据原路径补查。

## 可选设置

日常不需要加参数。终端飞行入口支持：

```bash
# 只在需要关闭记录时使用
roslaunch px4_wall_ceiling_control attachment_terminal_flight.launch record_logs:=false

# 只在需要改保存位置时使用；其余原飞行参数照常追加
roslaunch px4_wall_ceiling_control attachment_terminal_flight.launch \
  test_log_directory:=/home/你的用户/flight_records
```

统一系统入口同样支持这些参数。默认适用于现有生产话题；mock或重映射数据不会自动替换话题名。

## 实现与验证

仅新增`attachment_flight_logger.py`及必要的launch/安装配置。撤下上一版独立录制、标记、离线分析及机身元数据模板；理论讨论中的结构参数章节属于背景文档，不参与日志记录或控制。

测试覆盖自动目录/快照、程序日志复制、目标误差与速度坐标、异常停止和launch连接。真实ROS录制仍需要飞行机器短时验证。rosbag分段和roslaunch文件日志语义已核对[ROS Noetic rosbag源码](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosbag/src/record.cpp)及[roslaunch日志实现](https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/nodeprocess.py)。
