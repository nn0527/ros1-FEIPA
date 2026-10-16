# Early pilot handback after measured leveling

The user selected a tilt-only confirmation: actual total tilt <=5 degrees for
0.30 seconds. Angular-rate magnitude, horizontal/vertical speed magnitude,
height convergence, raw optical-flow reacquisition, and front-wall clearance
are not acceptance gates for this early handback.

The mode manager can request the configured wall pilot mode (POSCTL in the
console profile) directly from DETACH_ROTATE, RECOVER, ABORT or EXIT. This
avoids having to pass the decision node's full RECOVER/EXIT completion criteria
before the pilot can regain control. It does not label an incomplete detach or
an ABORT as a successfully completed maneuver.

Fresh connected/armed OFFBOARD feedback, valid RC input, clear emergency
interlock, current wall phase, active wall output ownership, and valid finite
odometry remain required. For POSCTL, PX4 must report valid horizontal velocity
and horizontal position estimation without constant-position fallback; attitude
and vertical velocity estimates must also be valid. Raw flow quality and speed
settling flags are not substituted for these PX4 requirements. There is no
automatic fallback to another pilot mode if POSCTL is unavailable.

Confirmation uses normalized quaternion thrust-axis tilt, not separate roll and
pitch limits. The odometry subscriber keeps only the latest queued message.
Each delivered odometry callback updates the confirmation under the same lock;
a brief tilt excursion between manager timer ticks restarts the window. Only
the manager timer requests mode changes.
An immutable message/receipt pair and integer source/receipt timestamps avoid
mixed callback data and loss of timing precision. At least three distinct
source samples and both source-time and receipt-time spans of 0.30 seconds are
required. Duplicate samples add no evidence; reversed stamps, gaps >0.20 seconds,
excess tilt or invalid required feedback restart the confirmation. A 1 ns
rounding tolerance accommodates ROS floating-time construction. New action
attempts cannot inherit a previous window.

The first qualifying manager tick requests POSCTL immediately. Requests are
retried every 0.50 seconds, with one request in flight at a time. The service
worker rechecks live handback conditions and action identity immediately before
the RPC. A successful RPC is only submission acknowledgement. Existing control
continues until fresh FCU mode feedback confirms POSCTL; then prestream ends,
the manager locks until the old operator request is released, and it does not
automatically re-enter OFFBOARD. A pilot-selected third mode is respected.

The console's existing phase/reason fields show the current blocker, the tilt
confirmation duration, pending mode confirmation, and completed pilot transfer.
ABORT remains visibly a fault outcome. No ROS message type or PX4 parameter is
changed. Existing attitude and thrust generation, emergency interlocks and
terminal deceleration are retained.

Configuration is under `flight_mode_manager_node` in `attachment_output.yaml`:
`wall_level_handover_enabled=true`, `wall_level_handover_tilt_deg=5`,
`wall_level_handover_hold_s=0.30`, `wall_level_handover_retry_s=0.50`.
Software tests do not establish actual mode-switch latency or flight stability.
