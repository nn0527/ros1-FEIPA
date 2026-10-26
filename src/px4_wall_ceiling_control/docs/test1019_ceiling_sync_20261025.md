# test1019 顶吸同步记录

日期：2026-10-25。

工作分支为 test1019，基线提交124495a；顶吸来源为test1024的144c509。本次仅修改本地工作区，未提交、推送或修改远程树莓派工作区。

## 同步内容

- 顶吸配置、状态/控制候选消息、决策节点、水平稳定控制组件、独立输出与状态监视。
- 与顶吸有关的POSCTL准入、OFFBOARD接管握手、接管姿态/推力连续性、恢复悬停和交接。
- 顶吸ABORT及HOLD超时请求AUTO.LAND的管理和输出链路。
- 顶吸控制台、网页、模拟反馈和对应回归测试。
- CH8选择顶吸/侧吸、CH12启动/脱离及launch参数，作为顶吸一致操作入口同步。该共用入口会影响遥控操作方式：默认CH8上拨为顶吸，下拨为侧吸；关闭rc_mode_selector_enabled可恢复旧的CH12仅侧吸入口。

顶吸主决策、配置、消息、运动组件、共用输出、操作入口等11个关键文件已逐字节核对，与test1024一致。

## 保留的test1019侧吸内容

不迁移test1024侧吸的自动退墙、融合竖直速度恢复门限及脱离ABORT变化。

侧吸决策节点、wall_perch.yaml、wall_detach_vertical.py、wall_simple_recovery.py、departure_height.py、simple_recovery.py六个文件保持基线原样；模式管理器20个_wall*/_captured_recovery*方法已比较AST，均与test1019基线一致。

共用模式管理器只同步顶吸及其接管握手相关差异，保留test1019侧吸姿态稳定后交回POSCTL的判断和提示。输出节点和消息中的新增顶吸字段用于顶吸链路。

## 验证结果及限制

- 顶吸与共用链路14个测试模块：329项，328通过、1项Linux Unix socket测试在macOS跳过。
- 顶吸界面：41条断言通过。
- test_attachment_safety：47项中41通过、2失败、4错误。六项均为侧吸旧测试；在未修改124495a源码的独立临时目录运行该模块，得到相同的六项失败/错误，未为本次同步改动侧吸程序或放宽测试。
- 已检查修改Python文件语法、launch XML解析、关键文件一致性和git diff --check。
- 离线测试使用基于消息定义的ROS替身，不代表ROS集成、catkin构建或飞行验证。

已尝试规定命令：

```bash
docker exec ros1_noetic bash -lc 'source /opt/ros/noetic/setup.bash && cd /root/ros_ws && catkin_make'
```

结果：本机返回`zsh: command not found: docker`。因此Docker构建验证未完成，未使用树莓派宿主机或其他宿主机编译替代；后续须在具备指定容器且挂载本次源码的环境完成构建。

离线测试日志：`/tmp/feipa-test1019-ceiling-sync-regression.log`；旧安全测试基线日志：`/tmp/feipa-test1019-baseline-tests.log`。
