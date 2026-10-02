# USB 飞控和传感器观察测试

原始压缩包：`C:\Users\60497\Desktop\catkin_ws.zip`。
桌面源码：`C:\Users\60497\Desktop\catkin_ws\src`。
Linux 编译工作区：`/home/wallrobot/catkin_ws_original`，Ubuntu-20.04 / ROS Noetic。
现有 robot 工程和 legacy 基线没有覆盖。原始包的历史 build.from_home_cry、devel.from_home_cry 仅为归档，不参与构建；不要 source 这些目录。

已完成：从压缩包提取源码，重新生成工作区顶层 CMake 链接，catkin_make 编译通过，原项目 60 项测试通过。原始包源码保持不变；本次新增的观察入口位于 runtime/。

## 本次实机结果（2026-10-01，香港时间）

USB：Auterion PX4 FMU v6X.x，VID:PID 3185:0035；Windows COM3 / BUSID 2-2；WSL `/dev/ttyACM0`。
15 秒采样：connected=true，armed=false，mode=AUTO.LOITER；state 16 条，IMU 752 条，local_position/odom 451 条。
顶部、前方 L1 均未连接，无数据，/sensor/health 两通道 valid=false。
这证明通信及这些消息的接收正常，尚未验证位置精度、测距传感器或控制算法。

启动链只有 MAVROS 和原始 sensor_range_manager，接传感器后增加原始 lidar_distance_node。不启动解锁、模式切换、控制输出或吸附机构节点。L1 驱动配置为 38400、l1_ascii，启动发送 iACM 测量命令。

## 再次运行（PowerShell）

先查询实际 BUSID，插拔后可能变化。飞控被转接到 WSL 时，Windows COM3 和地面站不能同时占用它。

```powershell
cd C:\Users\60497\Desktop\catkin_ws
& 'C:\Program Files\usbipd-win\usbipd.exe' list
powershell -ExecutionPolicy Bypass -File .\Connect-Usb.ps1 -BusId 2-2 -LinuxPort /dev/ttyACM0
powershell -ExecutionPolicy Bypass -File .\Start-Observe.ps1 -FcuPort /dev/ttyACM0
```

Start-Observe 持续运行；另开 PowerShell 检查输入：

```powershell
cd C:\Users\60497\Desktop\catkin_ws
powershell -ExecutionPolicy Bypass -File .\Check-Inputs.ps1
```

停止：在启动终端 Ctrl+C，或 `powershell -ExecutionPolicy Bypass -File .\Stop-Observe.ps1`。
USB 转交回 Windows：停止后执行 `& 'C:\Program Files\usbipd-win\usbipd.exe' detach --busid 2-2`。

## 接测距传感器

先用 usbipd list 识别每个 USB 串口适配器，分别 bind、attach 到 WSL，再查看 `/dev/ttyUSB*` 并确认顶部/前方的对应关系。Connect-Usb.ps1 可设置各自的 BUSID 与 LinuxPort。停止原观察入口后，按实际设备映射启动，例如：

```powershell
powershell -ExecutionPolicy Bypass -File .\Start-Observe.ps1 -FcuPort /dev/ttyACM0 -CeilingPort /dev/ttyUSB0 -FrontPort /dev/ttyUSB1
```

传感器口可单独留空，不会启动未连接的驱动。不要照搬原始 sensors.yaml 中另一台电脑的 by-path 串口路径。

## 日志和修改后编译

运行日志：`/home/wallrobot/catkin_ws_original/logs/`，构建和测试日志：工作区的 build.log、tests.log。验收快照：本目录 USB验证结果.json。

修改桌面源码后，仅同步 src 到 Linux，再重新编译；不要同步旧 build/devel：

```powershell
wsl -d Ubuntu-20.04 -- bash --noprofile --norc -c 'cp -a /mnt/c/Users/60497/Desktop/catkin_ws/src/. /home/wallrobot/catkin_ws_original/src/; source /opt/ros/noetic/setup.bash; cd /home/wallrobot/catkin_ws_original; catkin_make -j2 -l2'
```

USB 转接依据：[Microsoft WSL USB 文档](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)。本机另外已安装 kmod、usbutils，加载 cdc_acm，wallrobot 已加入 dialout。

## 实时数据可视化

飞控观察入口运行后执行：

```powershell
cd C:\Users\60497\Desktop\catkin_ws
powershell -ExecutionPolicy Bypass -File .\Start-Telemetry.ps1
```

打开 [PX4 实时遥测](http://127.0.0.1:8765/)。最新数值每50ms轮询，历史曲线每250ms刷新，保留最近60秒，以10Hz记录曲线。图例可隐藏某一轴，鼠标移动查看数值。接收频率按最近5秒的消息统计，与绘图采样频率不同。

姿态来自 IMU 四元数转换的 Roll/Pitch/Yaw，单位度；角速度由 rad/s 转为度/s。位置采用 odom 的 map 坐标，速度采用 odom 的 base_link 子坐标。IMU 加速度没有去除重力。过期数据在曲线中断开，不补零。

该服务仅订阅 ROS 数据，HTTP 仅提供读取接口，绑定本地地址，不发送飞控命令。启动终端 Ctrl+C 停止可视化，飞控观察进程保持独立。

电池消息当前返回 65.535 V 的原始读数。该读数对应 MAVLink 的 65535 mV 未提供标记，因此页面显示“电压未提供”，不当作实际电压。依据：[MAVLink SYS_STATUS 定义](https://mavlink.io/en/messages/common#SYS_STATUS)。遥控及测距通道未收到消息时显示“未收到数据”。

## 双路 L1 实机接收测试（2026-10-01）

本次连接映射：COM4 / BUSID 4-1 → `/dev/ttyUSB0`；COM5 / BUSID 4-4 → `/dev/ttyUSB1`。两路均为 CH340，已加载 ch341 驱动并授予 dialout 串口权限。顶部/前方物理对应关系尚未确认，按用户要求使用 COM4、COM5 标注。

在原飞控观察入口运行时执行 `powershell -ExecutionPolicy Bypass -File .\Start-Sensors.ps1`，然后启动 Start-Telemetry.ps1，打开同一实时页面。两路发布 `/sensor/com4/raw`、`/sensor/com5/raw`；原始 sensor_range_manager 以独立节点 usb_sensor_range_manager 运行，输出 `/sensor/com4/range`、`/sensor/com5/range`、`/sensor/usb_health`。health 消息沿用原始结构，ceiling 字段对应 COM4，front 字段对应 COM5，这仅是本次独立测试通道，不代表物理安装方向。既有 `/sensor/health` 属于原观察入口，尚未接入这两个测试通道。

初始 10 秒串口检查：COM4 93 帧，约 9.54 Hz，0.338–0.339 m；COM5 75 帧，约 7.89 Hz，0.097–0.103 m；均无解析错误。持续采样随后发现 COM4 间歇超时：11:23:31 的一分钟窗口有效绘图样本比例 40%，最长无有效数据段约 2.9 秒；COM5 同窗口有效样本比例 100%，约 8 Hz。数值是当前目标场景的接收结果，不是标定精度测试。

0.6 秒超时门限保持原配置。页面依据 SensorHealth 判定有效性，超时则隐藏当前距离并断开曲线，不延长有效期。启动时原始驱动两路各收到一次 BUSY 响应，之后继续收到测距帧；manager 的 invalid_count 统计越界/无效 Range 消息，不包含 UART 解码告警，不能将其解释为串口从未报错。

测试记录：runtime/lidar_serial_test.json、runtime/lidar_stream_test.json；页面快照：runtime/telemetry_snapshot.json。原始控制算法源码保持不变，本次只增加读取节点及图表。

停止传感器：启动 Start-Sensors 的终端 Ctrl+C；停止可视化：Start-Telemetry 的终端 Ctrl+C。再次插拔后先用 usbipd list 和 Linux ttyUSB 枚举确认端口映射，避免沿用已变化的编号。不要在传感器节点占用串口时运行 probe_lidar.py。

## 延时与稳定性优化

详见本目录《延时与稳定性测试策略.md》和《延时定位与优化结果.md》。页面改为IPv4地址、快速数值接口、图表本地依赖、锁外序列化和最新数据队列。测距原始驱动和0.6秒超时门限保持原配置。固定目标下基线已稳定约9.5Hz，后续优化主要减少数据进入页面后的显示等待。
