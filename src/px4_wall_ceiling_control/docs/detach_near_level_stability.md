# DETACH_ROTATE near-level output continuity

This patch is based on ai_correction2 d5e71eacb798f555950798afdbc7fc31353bd695.
The branch snapshot lacked the already approved bounded-height continuation fix.
That fix was restored from the verified 20261017 delivery before this change;
its original thresholds, budgets and completion gates are retained.

## Publication contract

In DETACH_ROTATE, the first eligible candidate immediately uses the existing
recovery limits: thrust-axis 10 deg/s, heading 20 deg/s, total tilt 5 deg.
Eligibility requires an active horizontal controller, fresh measured attitude,
and same-maneuver actual published history. Measured, published and unprojected
desired thrust axes must each be within 5 deg. A measured fallback seed or an
old action's history cannot establish the latch. The existing final thrust slew
(0.50 normalized units/s) and dt cap (0.05 s) remain in force.

The latch is committed only after final validity/authority checks and successful
rospy publisher handoff. This does not imply a PX4 acknowledgement. Candidate
creation or cone projection alone never updates published history or the latch.
Publication, maneuver lifecycle and state/odometry callbacks are serialized;
emergency/RC feedback remains live for the final authorization check.

Once latched, loss of horizontal-loop activity, temporary measured tilt outside
the cone or feedback restoration cannot restore the fast detach rate. Stale
feedback inhibits publication without erasing the latch. RECOVER uses the same
actual published history. If history has expired and the measured fallback is
outside the cone, output is explicitly refused instead of projecting it in one
step. New maneuver, retirement, ownership release and OFFBOARD exit clear the
latch and wall publication history. ABORT keeps immediate emergency leveling.

## Heading and diagnostics

Normal DETACH_ROTATE, RECOVER and EXIT attitude heading comes from q_hover.
Retreat world velocity still uses departure_yaw. Approach and emergency ABORT /
fault EXIT behavior are unchanged. No ROS message definitions or PX4 parameters
are changed.

/attachment/output_health adds near_level_latched_maneuver_id and, under
last_wall_command.publication_slew, rate_mode, latched_maneuver_id,
thrust_axis_step_deg and signed heading_step_deg. State reason and existing
asynchronous transition logs report rates xyz/norm, tilt, stable_s and failed
completion gates. These are diagnostic fields, not relaxed admission rules.

## Verification and physical acceptance

Regression tests exercise actual mocked ROS publication, including the saved
horizontal-braking entry at 2026-10-17 00:43:57.872665 Beijing time. Subsequent
50 Hz samples are explicitly synthetic and prove output bounds only. The
9,352,766-byte ULog log41 has SHA256
`a42889927487d5445fca23c1db922379f06aee22d723b8724a6bb1fb2302afcd`, no recorded
dropouts. Its already-published PX4 setpoints cannot reconstruct missing raw
ROS controller candidates.

Build and run registered tests inside ros1_noetic:

```bash
source /opt/ros/noetic/setup.bash
cd /root/ros_ws
catkin_make
export PYTHONPATH=/root/ros_ws/src/px4_wall_ceiling_control/test:$PYTHONPATH
export NOSE_NOLOGCAPTURE=1 NOSE_NOCAPTURE=1
catkin_make run_tests_px4_wall_ceiling_control
catkin_test_results --verbose build/test_results/px4_wall_ceiling_control
```

The baseline has pre-existing test failures and a nosetests async-logging
shutdown hang; see the delivery report for before/after results and any bounded
test-runner termination. Do not interpret a build pass as all tests passing.

Physical success still requires trajectory completion, restored height loop,
measured tilt <=5 deg and three-axis rate norm <=10 deg/s for >=0.30 s, and an
actually published target inside5 deg, within the original8 s deadline.
Human testing must first verify stable hover, then record three consecutive
qualifying detach/recover runs with the tether not continuously carrying load.
Keep current PX4 parameters and retain failed logs for inner-loop diagnosis.
Software replay does not establish this physical acceptance.
