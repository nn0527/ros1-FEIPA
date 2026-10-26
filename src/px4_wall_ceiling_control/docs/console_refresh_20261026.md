# 顶吸与侧吸测试台更新（2026-10-26）

以 test1019 的 e33ba4e 为基线，调整 ceiling_wall_console 页面及状态序列化。

桌面顶吸、侧吸同一行，顶吸区较宽；侧吸定高和斜面操作收入折叠区，保留全部现有操作入口。顶部汇总实时飞行反馈，各任务在自己的卡片内显示执行状态。手机宽度下自动上下排列。

顶吸新增节点实际 HOLD 计时、气压下降达标、人工/自动协调和恢复交回反馈。保持倒计时仅显示节点反馈，不在浏览器推断降落已经执行；AUTO.LAND 仍以实际飞控模式区分请求中/已确认。显示新增遥控协调字段；缺失时提示等待，过期/暂停/断连时不显示为有效。后端透传 rc_manual_coordination_enabled、rc_manual_feedback_ready、rc_manual_active、rc_position_weight。

按当前正常侧吸程序修正文案：捕获恢复姿态和高度闭环，无自动退墙，第三次 CH12 确认交回。顶吸不采用第三次确认规则。共用管理器 PRESTREAM、ENTERING、RETURNING、LOCKED 只影响当前所属任务，避免侧吸状态覆盖顶吸状态。

验证：既有 ros1_noetic 容器内完整 catkin_make 通过；108 项控制台 Python 测试通过；全部5组网页测试通过（含新增顶吸计时、气压无效、遥控协调、反馈过期和任务隔离断言）。离线浏览器检查保持、下降场景，桌面两栏等高起点、侧吸宽度较小；390px 手机宽度无横向溢出。正式8790端口认证后的观察会话确认页面已更新、实时反馈服务已连接，未申请控制权或发送动作。

重新加载 wall_rc_startup 的共享反馈服务以应用序列化字段，ceiling_wall_console HTTP 服务保持运行。未重启飞行控制链路，未做实飞。若控制节点仍为旧消息版本，应在地面按原流程重启基础链路后再读取新增字段。

日志（容器）：/tmp/test1019-console-refresh-build.log、/tmp/test1019-console-refresh-tests.log；网页测试日志（树莓派）：/tmp/test1019-console-refresh-ui.log。
