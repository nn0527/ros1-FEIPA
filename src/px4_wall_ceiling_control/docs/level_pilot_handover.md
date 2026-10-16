# Fast handback after leveling and height/velocity convergence

Automatic wall-flight handback to POSCTL is permitted only in RECOVER, ABORT,
or EXIT. DETACH_ROTATE cannot accumulate confirmation or send a POSCTL request.
The following conditions must hold together for 0.20 seconds:

- Actual total thrust-axis tilt <=5 degrees.
- Absolute height error to the existing recovery controller reference <0.20 m.
- Absolute ENU vertical speed <0.30 m/s.
- ENU horizontal speed magnitude <0.40 m/s.

Height and speed handback tolerances are independent manager parameters.
The decision node's existing recovery settle and height-continuation settings
are retained, including the 30-degree continuation entry cone.
The manager does not capture a new height reference or change the height
controller, thrust generation, reference reanchoring, or continuation logic.
It requires a valid reference and an active restored height loop. Height error
and world-frame velocity use the same odometry snapshot.

No additional angular-rate, front-clearance, or old one-second horizontal
stability gate is added. The existing DETACH_ROTATE -> RECOVER gate is retained.
ABORT remains a fault outcome, not successful detach completion. The first
qualifying manager tick requests POSCTL; no extra RECOVER dwell or completed
EXIT is needed. Approach-timeout returns use the same height/velocity criteria.

Fresh connected/armed OFFBOARD feedback, valid RC input, clear emergency
interlock, current wall phase, active wall output ownership, and valid finite
odometry remain required. For POSCTL, PX4 must report valid horizontal velocity
and horizontal position estimation without constant-position fallback; attitude
and vertical velocity estimates must also be valid. Raw flow quality and speed
settling flags are not substituted for these PX4 requirements. There is no
automatic fallback to another pilot mode if POSCTL is unavailable.

Confirmation uses normalized quaternion thrust-axis tilt, not separate roll and
pitch limits. The odometry subscriber keeps only the latest queued message.
Odometry and wall callbacks update confirmation under the same lock; a brief
height, speed, tilt, or reference-validity excursion between manager timer ticks
restarts the window. A changed height reference also restarts confirmation.
Only the manager timer requests mode changes.
An immutable message/receipt pair and integer source/receipt timestamps avoid
mixed callback data and loss of timing precision. At least three distinct
source samples and both source-time and receipt-time spans of 0.20 seconds are
required. Duplicate samples add no evidence; reversed stamps, gaps >0.20 seconds,
failed convergence or invalid required feedback restart the confirmation. A 1 ns
rounding tolerance accommodates ROS floating-time construction. New action
attempts cannot inherit a previous window.

The first qualifying manager tick requests POSCTL immediately. Requests are
retried every 0.50 seconds, with one request in flight at a time. The service
worker rechecks live handback conditions and action identity immediately before
the RPC. Both scheduling and the worker enforce the phase whitelist. A successful
RPC is only submission acknowledgement. Existing control
continues until fresh FCU mode feedback confirms POSCTL; then prestream ends,
the manager locks until the old operator request is released, and it does not
automatically re-enter OFFBOARD. A pilot-selected third mode is respected.

Canceled OFFBOARD entry has a matching recovery path: an explicit wall cancel
with fresh CANCELING_ENTRY, armed OFFBOARD, and valid odometry lets the idle
wall decision report ABORT. The existing owner-zero cancellation controller
continues to publish without a new WALL acquisition. For this case the manager
checks the same convergence gates against the mux's frozen
`cancel_vertical_control.height_reference_m`, requiring fresh active output
health. A fresh FCU state already in POSCTL may reassert that same mode to
clear an old OFFBOARD intention; this is not a transfer out of OFFBOARD.

The console's existing phase/reason fields show the current blocker, the joint
confirmation duration, pending mode confirmation, and completed pilot transfer.
ABORT remains visibly a fault outcome. No ROS message type or PX4 parameter is
changed. Existing attitude and thrust generation, emergency interlocks and
terminal deceleration are retained.

Configuration is under `flight_mode_manager_node` in `attachment_output.yaml`:
`wall_level_handover_enabled=true`, `wall_level_handover_tilt_deg=5`,
`wall_level_handover_hold_s=0.20`, `wall_level_handover_retry_s=0.50`,
`wall_level_handover_height_error_m=0.20`,
`wall_level_handover_vertical_speed_mps=0.30`,
`wall_level_handover_horizontal_speed_mps=0.40`.
Height and speed comparisons are strict; equality does not pass.
Software tests do not establish actual mode-switch latency or flight stability.
