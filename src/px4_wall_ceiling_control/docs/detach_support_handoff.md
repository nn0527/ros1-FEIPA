# test1019：承重交接与测距短时支撑

针对 2026-10-23 12:17 真机日志 `20261023T041541Z_fcc0d5` / `log61`：
压墙期间机体被推高，旧高度参考使修正达到负限幅；转为自主承重时减推力，
测距锁定后无速度 hold 又保持低于悬停的实际输出，随后下降保护触发 ABORT。

## 控制变化

- 测距锁定前，按零修正的悬停基准预计算承重权重，也检查原需求的权重。
  仍需墙面承重时，高度参考跟随实测，清积分、目标速度和整个高度/速度修正为零。
  仅清积分不足以消除上升速度对应的负比例修正，因此必须同时抑制该项。
- 实际发生过承重跟随后，首次全承重拍继续跟随一次，需求为倾角补偿后的悬停前馈。
  后续全承重拍恢复原高度/速度反馈。直接水平初始化继续保留原高度目标。
- 控制器提供 `reference_tracking`；现有 `detach_vertical_reason` 在跟随期间附带
  `reference_tracking=True`，并同步脱离/恢复高度引用，不改变 ROS 消息接口。
- 已锁定的测距闭环显式关闭参考跟随，保留测距固定目标。
- 快速速度暂缺与测距短时失效两处 hold 共用实现：目标为
  `min(max_thrust, max(actual, base/max(vertical_fraction, 0.001)))`。
  从新鲜、有效的实际输出按原 `0.5/s` 限速，每步时间最多 `0.05 s`；
  同一时刻重复求值不重复增加推力。低输出会向悬停恢复，不保证瞬间达到悬停。
  目标速度仍为 NaN，不用高度误差驱动 hold，不伪造速度。

下降保护 `max_down_velocity_mps=0.80`、ABORT 回平、急停/权限、新鲜度、
短时支撑超时和 `_detach_rotation_ready` 均沿用原实现。
本次不处理高度振荡包络或组 1 退墙串行化。

## 回归与证据边界

`test_detach_support_handoff.py` 覆盖承重上升、部分权重、交接、无速度 hold、
倾角补偿、速率/上限/同拍幂等、权限和超时，以及 log61 的真实观测回放。
夹具 `test/fixtures/detach_support_log61.json` 保留来源 SHA256、源/接收时间、
原始 odom、测距和记录输出。证据的 `28.5–32.6 s` 以北京时间 12:16:40 为零点，
实际对应 12:17:08.5–12.6。

回放使用真实运动输入，并将上一条新指令作为下一拍实际输出回读的理想软件模型。
它检验指令策略，不重新模拟飞行动力学；原日志的下降速度和 ABORT 不能据此改写。
正常脱离能否回正进入 RECOVER，仍须新试飞日志确认。

旧断言仅更新 `FastRangeVelocityControlTests.test_nan_expired_and_future_fast_samples_cannot_drive_control`：
无有效快速速度且原实际输出为 `0.50` 时，原断言维持 `0.50` 改为在已有时间预算内恢复至
悬停基准 `0.518`；高度、速度错误均不参与 hold。原 `0.61/0.68` 高于悬停的 hold 继续保持。
`test_height_continuation`、`test_detach_vertical*` 与 `test_bounded_detach_integration`
原断言保留。
