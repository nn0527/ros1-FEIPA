# test1019 斜面吸附与脱离实现（2026-10-26）

本次修改位于本机 `test1019`，保留已同步的顶吸内容。适用场景为静止平整斜板下表面、坡度约 30–60°、机身随板面倾斜，依靠推力压紧及车轮摩擦承重。按本次讨论，以顶部雷达 ≤6.5 cm 作为压实信号。车轮接触面与桨叶旋转平面垂直相距 5 cm；该尺寸属于机械条件，程序不据此计算未知坡度。

## 模块与复用

新增独立 `InclineAttachment` 状态机及 `InclineAttachmentMixin` 适配层，入口为 `OperatorCommand.MODE_INCLINE=5`。状态机运行在已有侧吸节点内，复用侧吸的状态话题、候选输出、控制权仲裁及唯一 MAVROS 输出链；候选消息携带 `incline_active`，防止侧吸与斜面请求混用。没有新增第二个飞控设定值发布器。

复用已有四元数几何、`ReturnAxis`、`BrakingTrajectory`、制动距离函数、`DepartureHeightController`、水平速度反馈与接近控制、OFFBOARD 接管及 POSCTL 交回流程。回平规划从最终实际发布的姿态和推力起步，并通过两个不同时间的输出样本估算入口角速度；初始规划角加速度取零。支撑控制采用高度与竖直速度 PD 及姿态补偿。

斜面不调用侧吸的固定 90° 翻转、侧墙测距脱离任务，也不调用顶吸的接触与脱离状态机。普通侧吸、顶吸仍使用各自模式。所有斜面算法参数独立放在 [`config/incline_attachment.yaml`](../config/incline_attachment.yaml)，不借用侧吸推力上限。

## 正常流程

| 阶段 | 进入及行为 | 下一步 |
| --- | --- | --- |
| WAIT | 从 POSCTL 请求斜面；前距 ≤50 cm 持续 0.4 s，同时满足姿态、速度、顶距、水平接管、已有高度及健康门槛 | 由现有管理器接管 OFFBOARD |
| APPROACH | 以 0.10 m/s 低速接近，保持高度及水平反馈 | 前距 ≤10 cm |
| SLOW_FLIP | 停止水平速度闭环；机尾下转，缓慢增加俯仰目标及归一化总推力 | 顶距 ≤6.5 cm |
| CONTACT_SETTLE | 立即停止翻转及增推，冻结接触时实际发布的总推力；姿态目标从最后输出平滑过渡到实测贴合姿态 | 姿态过渡完成后，连续有效接触与稳定反馈持续 0.2 s |
| HOLD | 保持贴合姿态和总推力基准 | 接收脱离命令 |
| RETURN | 从实际最终输出连续回平，限制角速度、角加速度、姿态跟踪误差及推力变化 | 实测及目标回平、角速度稳定 |
| RECOVER → EXIT | 恢复水平速度控制，确认近水平、低速度及高度稳定 | 交回 POSCTL |

HOLD 固定的是姿态和总推力基准，PX4 姿态闭环继续分配各电机输出，不冻结四路电机值。回平后不再要求顶部雷达保持近距。有效样本确认使用源时间戳，重复帧不累计稳定时间。

该实现适应俯仰方向的未知坡度；仍需机头大致朝向斜面下端接近。尚未实现依靠两个单点雷达自动搜索坡向或修正任意左右偏斜。

## ABORT

| 触发阶段 | 处理 |
| --- | --- |
| APPROACH | 取消向前接近目标，进入连续回平与悬停恢复流程；水平速度在恢复阶段收敛 |
| SLOW_FLIP / CONTACT_SETTLE | 停止继续翻转、增推，从最后实际输出受控回平 |
| HOLD | 脱离接触并受控回平；持续丢距或姿态不稳也触发此路径 |
| RETURN / RECOVER / EXIT | 保留现有回平与恢复时间预算，重复取消不重新计时 |
| 关键反馈失效、最终推力越界或回平/恢复超时 | 标记 LAND 并请求已有飞控故障/降落交接；不置零推力、不瞬间给水平姿态 |

遥控器或飞控已切离 OFFBOARD 时释放旧任务；再次出现 OFFBOARD 不自动续跑旧轨迹。LAND 是故障交接请求，是否完成模式切换取决于飞控和既有交接链反馈。

## 新增参数与初始限制

完整参数名及默认值见 YAML。关键参数如下，均位于 `/incline_attachment/`：

| 参数 | 默认值 / 用途 |
| --- | --- |
| `enabled` / `flip_thrust_max` | `false` / `0.0`；必须标定最大总推力并显式启用 |
| `front_ready_distance_m` / `flip_trigger_distance_m` / `top_contact_distance_m` | `0.50` / `0.10` / `0.065` m |
| `horizontal_approach_speed_mps` / `approach_timeout_s` | `0.10` m/s / `20` s |
| `flip_max_tilt_deg` | 目标及实测倾角上限 `65°` |
| `flip_pitch_rate_deg_s` / `flip_pitch_accel_deg_s2` | `10°/s` / `10°/s²` |
| `flip_thrust_ramp_per_s` / `flip_duration_timeout_s` | `0.03/s` / `8` s |
| `flip_max_tracking_error_deg` | `10°`；跟踪落后暂停继续翻转、增推，超时继续计时 |
| `hold_entry_blend_s` / `top_contact_hold_s` / `contact_settle_timeout_s` | `0.5` / `0.2` / `1.0` s |
| `hold_contact_loss_margin_m` / `hold_contact_loss_hold_s` | 比接触阈值远 `0.015` m，持续 `0.3` s 触发退出 |
| `detach_max_rate_deg_s` / `detach_max_accel_deg_s2` | `30°/s` / `30°/s²` |
| `detach_duration_timeout_s` / `recover_timeout_s` | 回平 `10` s / 悬停恢复 `15` s |
| `return_thrust_slew_per_s` | `0.20/s`，限制回平及恢复推力变化 |
| `detach_brake_*` / `detach_recovered_*` | 回平末段制动及稳定确认 |
| `height_kp` / `velocity_kp` | 均为 `0.30`，用于支撑恢复 PD |

触及角度、推力或时间上限仍未接触时触发 ABORT；接触与上限恰好同时达到时优先接触，实际已超出硬限则不接受接触。输出仲裁器另外限制最终发布倾角、角速度和推力变化。

这些数值是初始软件约束，离线测试不构成实机标定。`flip_thrust_max` 必须高于接管时总推力并不超过统一输出上限，当前未提供可直接用于飞行的标定值。

## 使用与验证

1. 完成推力上限标定，修改 YAML 中的 `flip_thrust_max`，再设 `enabled: true`；保留既有侧吸节点的 `auto_approach` 和 `horizontal_velocity_enabled` 开启。
2. 重新编译消息并重启整条控制链，确保各节点加载一致参数；参数不支持在动作中热切换。
3. 飞行至稳定 POSCTL，满足已有接管条件后，点击页面“斜面吸附”或向 `/attachment/terminal_command` 发送 `incline`。
4. HOLD 后用原有“脱离”按钮、`detach` 命令或 CH12 触发脱离。CH8 继续只选择原有顶吸/侧吸，不新增斜面档位。`cancel` 进入上述 ABORT 路径。

新增测试覆盖理想闭环反馈下的 30°/45°/60° 接触及回平、接触推力锁定、范围边界、重复帧、超时、重复 ABORT、输出连续性、模式隔离及入口/脱离交互。它们通过本机离线 ROS 消息替身执行，不代表真实 ROS 运行或飞行动力学验证。

最终相关离线回归共 357 项：356 项通过，1 项因需要 Linux Unix socket 跳过；页面 41 项断言通过，Python/JavaScript 语法、launch XML、斜面 YAML 及 `git diff --check` 通过。该回归未纳入已有的 `test_attachment_safety` 全套：其 2 个失败及 4 个错误已在修改前 `124495a` 基线复现，属于侧吸原有问题。

Docker 编译按工作区要求尝试，但本机没有 `docker` 命令，尚未完成 catkin 编译；没有改用树莓派宿主编译，也没有部署至树莓派。
