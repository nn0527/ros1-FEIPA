# Codex 本机贴墙试飞页面

页面在 Windows 本机运行，Codex 内嵌浏览器打开 `http://127.0.0.1:8787`。
本机后台通过免密 SSH 连接 `pi@192.168.50.66`，在 `ros1_noetic` 容器中启动
ROS 基础程序、发布动作请求、订阅真实反馈。树莓派不开放 HTTP 端口。

## 打开页面

在 Windows 工作区执行：

```powershell
.\Start-WallConsole.ps1 -NoBrowser
```

然后在 Codex 浏览器打开 `http://127.0.0.1:8787`。不传 `-NoBrowser` 时打开系统浏览器。
启动脚本优先使用 Codex 自带 Python（包含 Paramiko），避免使用旧版系统 Python。

前提：已验证的 SSH 公钥与 known_hosts、树莓派上的 `ros1_noetic` 容器、
`/root/ros_ws/devel/setup.bash`，以及同步到容器工作区的本包源码。
SSH 主机、用户、密钥和本机端口可通过脚本参数修改。

## 本次测试的操作

1. 飞控未解锁时，检查页面的三路串口和波特率，点击 **启动基础程序**。
   默认沿用这次日志中的 `/dev/ttyUSB0:921600`、顶部 `/dev/ttyUSB1`、前向 `/dev/ttyUSB2`。
   悬停推力初值为 0.45；请使用本机标定值。配置保存在本机
   `~/.codex/wall-console/settings.json`，不修改飞控参数。
2. 启动程序仅开启 ROS master（若需要）、唯一 MAVROS、雷达、贴墙决策、模式管理、
   唯一输出和自动记录节点；不会解锁或自动起飞。飞手在 ALTCTL 起飞并稳定。
3. 就绪检查通过后，点击 **一键贴墙**。桥接发布一个 `std_msgs/String(data='wall')`
   到 `/attachment/terminal_command`，等价于原来的 `rostopic pub -1`。
   页面不自动重发命令，也不自动重放断连期间的操作。
4. 观察接近、翻转、确认接触和贴墙保持。只有新鲜的 `WALL_HOLD`、`ACTIVE`、
   wall owner、已连接且已解锁的 OFFBOARD 同时存在，才显示保持确认。

专用 `wall_console_flight.launch` 复用 `attachment_system_mavros.launch`，显式开启
flight mode/output/wall，关闭可选 RC 姿态修正与 RC 中位交接要求。因此本次只贴墙测试
不需要配置 RC 通道索引；原有 `attachment_terminal_flight.launch` 不变。
新入口仅启动 wall 决策，两路雷达均保留，贴墙保持不定时自动脱离。

2026-10-02 已纳入树莓派现场调整的 `config/wall_perch.yaml`：翻转触发距离
`flip_trigger_distance_m: 0.12`，顶部接触距离 `top_contact_distance_m: 0.056`。
页面启动入口直接加载此文件，不覆盖这两个参数。

基础程序可在静态、未解锁状态启动并检查连接、传感器和反馈；此时一键贴墙未就绪
属于正常状态。实际交接 OFFBOARD、翻转和贴墙保持需要起飞稳定后验证。

## 页面反馈与限制

- SSH、飞控连接、解锁、实际模式、输出 owner、控制阶段和原始故障原因。
- 两路有效距离、实测横滚/俯仰、ENU 竖直速度、电池电压。
- 真正发布到 `/mavros/setpoint_raw/attitude` 的推力；不把候选值或飞控回传当作已发送值。
- 心跳、里程计、估计器、RC、雷达和贴墙状态的数据年龄。超过现有控制链路的阈值会标为过期，
  不使用历史值确认成功。页面的就绪检查不是完整飞行资格认证，飞手判断稳定后再点击。
- 操作回执表示发送完成；订阅的控制状态表示节点确认与执行进展。

启动幂等，不同时创建两个 MAVROS/控制链路。原有其他控制入口仍在运行时显示冲突，
不会自动结束它们或抢占串口。配置变更不支持在线重启控制链路。
切换旧启动入口时，应在飞控未解锁状态用 Ctrl+C 结束整个旧 roslaunch，不能只结束
ROS master：残留节点可能不再出现在节点列表中，却仍持有雷达串口。
页面启动在确认飞控未解锁后，检查两路雷达没有其他串口读取进程，再发送一次
`iHALT` 结束此前的测量，新雷达节点用 `iACM` 重新启动；避免热启动时返回 `BUSY`
而没有有效距离。重复点击已运行的基础程序不复位雷达，运行中也不自动复位。
页面关闭、后台退出或 SSH 中断不会主动关闭已经启动的飞行进程，避免网络故障改变控制输出。
SSH 恢复只恢复订阅。程序退出后页面会取消就绪状态，并显示缺失节点。
本页面没有解锁、停机、模式强切或“急停”按钮；结束测试仍由飞手按原流程接管。

树莓派容器中的启动日志位于 `/root/.ros/wall_console/`；自动试飞记录仍在
`/root/wall_test_records/`。本机服务日志在 `%LOCALAPPDATA%/WallConsole/`。

## 验证

核心状态检查可在无 ROS 的 Python 环境运行：

```bash
python3 -m unittest discover -s src/px4_wall_ceiling_control/test -p test_wall_console.py
```

前端测试使用模拟 HTTP 反馈验证未就绪、就绪、请求回执、贴墙确认、故障和 SSH 中断。
真实 SSH/ROS 连接验证不发送贴墙请求或启动飞行控制；实际离地和贴墙必须现场测试。
