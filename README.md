# catkin_ws

ROS1 Noetic workspace for PX4 wall/ceiling attachment control.

The local SSH flight console and instructions for using another computer are in
[`wall_console.md`](src/px4_wall_ceiling_control/docs/wall_console.md).
Windows operators can double-click the portable `WallConsole.exe`; Codex and Python are not required.
Developers build it with `./Build-WallConsole.ps1`. Source startup remains `./Start-WallConsole.ps1`.

## Packages

| Package | Responsibility | Primary documentation |
|---|---|---|
| `px4_wall_ceiling_control` | Dual range sensors, ceiling/wall decisions, owner arbitration and MAVROS output | [`src/px4_wall_ceiling_control/README.md`](src/px4_wall_ceiling_control/README.md) |

Production logic belongs inside a ROS package. Workspace-root files are limited
to workspace metadata and this index; test procedures live beside the package
that owns them.

## Build

```bash
cd /home/cry/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

The main bench procedure is
[`bench_test_procedure.txt`](bench_test_procedure.txt).

## Collaboration

Repository: https://gitee.com/yls2026/wall-climbing-robot

Create a working branch from `develop` and submit a pull request back to
`develop`. The initial workspace import is on
`import/catkin-workspace-20261002`.

```bash
git clone git@gitee.com:yls2026/wall-climbing-robot.git catkin_ws
cd catkin_ws
git switch develop
git switch -c your-working-branch
```

Build outputs, Python environments, logs, ROS bags and generated telemetry are
excluded from Git. The PowerShell and `runtime/` observation tools contain
paths for the original Windows/WSL installation; adjust those paths for your
machine before using them.
