# CH12 回弹拨杆与网页混合操作

2026-10-25，树莓派 `/home/pi/ros_ws`，分支 `test1019`。

实测 `/mavros/rc/in` 有18个通道，CH12为数组索引11。原位约999，拨动约1999，松手返回999。

`wall_console_flight.launch` 与 `attachment_terminal_flight.launch` 默认启用
`rc_wall_toggle_enabled:=true`，通道参数 `rc_wall_channel_index:=11`。
通用 `attachment_system_mavros.launch` 默认关闭此功能。
需要关闭时，在上述入口追加 `rc_wall_toggle_enabled:=false`。

- 飞控已连接、已解锁，飞控和墙面状态反馈新鲜，里程计有效，墙面为IDLE且操作意图待命时，拨动请求wall。原有飞控模式、测距、权限和进入条件继续由现有链路检查。
- 达到WALL_HOLD后再拨动，请求detach。靠墙、翻转、脱离、恢复、吸顶等其他动作中不接受CH12事件。
- 遥控器、网页、终端共用TerminalOperatorNode的操作意图及命令处理。支持遥控器启动后网页脱离，或网页启动后遥控器脱离。
- 回弹只允许下一次触发，不撤销动作。低位<=1200、高位>=1800，两端各连续至少0.05秒；每次拨动仅触发一次。
- 启动时拨杆在高位、通道缺失、非法输入、RC消息间断超过0.5秒或时间倒退，都必须重新稳定回低位，才允许后续高位触发。
- 脱离条件未满足或请求超时后，状态回报仍为HOLD且没有待处理请求时，可重新拨动或使用网页重试。
- 急停及模式接管继续沿用现有链路。未添加解锁、自动起飞或新的飞控设定值发布入口。

需要重启控制链路才能加载新代码。此次只做编译与测试，没有启动实飞控制链路。

## 12:46 飞次启动互锁修复

当时处于 POSCTL、已解锁、急停有效且未触发，三次 CH12 事件都被识别。
但 IDLE 状态的 fault_detected 包含 require_offboard 对当前模式的判断，
所以在 POSCTL 下为 true，旧 CH12 入口据此拒绝启动。
现将 IDLE 的聚合故障检查改为明确的里程计新鲜检查；连接、解锁、
飞控及贴墙状态时效、急停、规定的飞手模式检查保留。
模式管理器继续检查估计器、水平速度、接管平衡基准、高度及预发送等条件。
HOLD 阶段的故障检查保留，不允许故障时由 CH12 发起正常脱离。

操作顺序（wall_console_flight.launch）：

1. 正常启动基础程序，在 POSCTL 中由飞手起飞。CH12 先回低位。
2. 拨动 CH12 并松手，发出 wall 请求。接管条件满足后自动进入 OFFBOARD，
   缓慢接近，再按有效前向距离触发翻转和贴墙保持；等待条件不满足时不会强行接管。
3. 到 WALL_HOLD 后再次拨动 CH12 并松手，请求脱离；符合现有脱离条件后
   执行 DETACH_ROTATE、RECOVER、EXIT，并在管理器确认安全交接后返回 POSCTL。
4. 确认 POSCTL 反馈和贴墙节点回到 IDLE 后，可再次拨动开始下一轮。

接近、翻转、脱离、恢复中重复拨动不会跳阶段，也不会重发脱离请求。
记录每次被忽略事件的操作模式、贴墙阶段、待脱离、权限、故障标志及原因，
便于区分未启动和已启动但等待接管条件。

新增回归覆盖 POSCTL 的 IDLE 聚合故障、无效里程计、错误起始模式、急停无效、
断连、过期/未来反馈、HOLD 故障，以及一次完整遥控意图周期和重复周期。
针对测试共 11 项；连同接近、翻转、脱离、恢复、控制权交还及急停测试共 284 项通过。
使用当次 rosbag 中三次 CH12 事件之前的实际反馈离线重放，旧入口三次拒绝，
修正入口三次均接受 wall 请求。该验证仅证明软件入口及回归行为，不代表实飞通过。

验证：在ros1_noetic容器内执行规定的完整catkin_make成功；
新增test_rc_wall_toggle的6项测试通过，原test_flight_handover、test_wall_console、
test_attachment_flight_logger共127项测试通过。两个launch展开后的参数均为启用、通道11。
实飞动力学效果尚未在本次修改中验证。

切换分支前，原test1024的7个已跟踪文件修改已保存为Git stash：
`On test1024: Preserve test1024 tracked changes before CH12 test1019 integration`。
另有补丁 `/home/pi/codex_backups/ch12-before-test1019/test1024-working.patch` 和状态清单。
原未跟踪备份文件保留在原处。
