# 脱离高度异常的有界续接

基线：树莓派 `ai_correction`，`d5e71eacb798f555950798afdbc7fc31353bd695`。
本变更不修改消息定义、不改变翻转轨迹或正常完成的姿态门槛。

## 采样和异常检测

决策定时周期与里程计/飞控状态回调共用锁；一个周期使用同一份里程计消息及接收时间。
里程计订阅队列为 1；沿用源时间新鲜度和乱序拒收检查。
高度残差在状态机决策前计算：

`abs(z_new - z_old - 0.5 * (vz_new + vz_old) * (source_new - source_old))`。

速度为转换到 ENU 后的竖直速度。残差大于 0.25 m 时，只有实测倾角不超过 30°、姿态/估计器/OFFBOARD/实际输出有效，才进入受限支撑。
原急停、下降速度、输出权限、8 秒脱离超时和离墙确认仍生效。

## 续接与恢复

- SUPPORT：关闭高度位置与竖直速度纠偏、清零积分；用本次已捕获垂直支撑基准和实测倾角，沿用压力种子及补偿算法。每步从可信实际输出按原 0.50/s 限速，沿用总推力上限。
- 单次 1.2 秒、同一次脱离累计 1.5 秒；确认窗口和平滑恢复均计入。重复异常、RECOVER/EXIT 转换不重置预算。
- 确认要求：连续源时间及本地观察时间均至少 0.30 秒，至少 4 个不同源样本；间隔≤0.20 秒，单步残差≤0.05 m，窗口累计绝对残差/时长≤0.20 m/s，倾角≤10°，|vz|≤0.20 m/s。
- 不合格样本清除确认窗口；重复/乱序样本不能累计证据。
- 确认后同时更新控制器、脱离和恢复高度基准，保留支撑基准、压力种子、实际输出与轨迹进度。
- BLENDING：0.30 秒恢复全部闭环纠偏，期间不积分。再次异常继续使用原预算；失败进入 ABORT。
- 只有高度闭环完全恢复、原轨迹完成、实测倾角≤5°且角速度≤10°/秒持续≥0.30 秒、发布目标也在5°内，才允许 DETACH_ROTATE→RECOVER。
- 正常 EXIT 在交还遥控器前仍可撤销完成标记；故障恢复 EXIT 保持原故障恢复输出，不能冒充正常脱离成功。

`WallPerchStatus.detach_vertical_reason` 记录暂停/恢复、残差、单次与累计时长、纠偏权重和新旧高度基准。`vertical_control_active` 在正常续接/混合期间为 false，`recovery_support_valid` 可为 true，表示受限支撑有效。

## 回放证据与测试边界

`test/fixtures/detach_height_fault_20261016.json` 来自 `20261016T152922Z_27d0fe` 的原始里程计，含原四元数和机体系速度；ENU 速度由原始四元数旋转得到。
故障前后源时刻：1792164635.0899413 → 1792164635.1699755；高度8.20760536→7.99732447 m，ENU vz 1.036993395→0.841545485 m/s，残差0.285454626 m。

集成回放保留原始高度、ENU速度、倾角、角速度大小及时间间隔；使用可信输出回传夹具，不是飞行动力学仿真。
测试中的稳定恢复后缀明确为合成数据：固定高度、水平姿态、零速度及角速度。它验证合法进入 RECOVER；原始不稳定后缀仍拒绝正常完成。

新增27项测试覆盖正负跳变、窗口/预算、重复乱序/突发样本、回调竞争、权限/反馈失效、积分/三个基准/输出连续性及正常和故障交接。两个旧坐标跳变测试改为31°实测倾角，以继续验证允许续接范围外的拒绝路径。

Docker中复现：

```sh
source /opt/ros/noetic/setup.bash
source /root/ros_ws/devel/setup.bash
cd /root/ros_ws/src/px4_wall_ceiling_control
PYTHONPATH="$PWD/test:$PYTHONPATH" python3 -m unittest test_height_continuation test_bounded_detach_integration test_detach_vertical_authority test_detach_vertical test_detach_state_machine test_detach_output_continuity -v
PYTHONPATH="$PWD/test:$PYTHONPATH" python3 -m unittest discover -s test -p 'test_*.py' -v
```

修改前全量667项，6失败/7错误；修改后694项，仍为同名6失败/7错误，没有新增失败。全量测试并非全绿，具体原有失败见交付验收记录。
软件回放与编译不能证明真实飞行成功，仍需新飞行日志确认连续输出和合法状态转换。
