# Detach terminal target deceleration

The two records `20261016T180828Z_f30651` and
`20261016T181027Z_81e363` showed actual rearward tilt of about 10 degrees
after the requested attitude had already reached level. The existing
near-level limiter waited for an active horizontal velocity controller;
it therefore did not slow the first crossing of level.

This change adds a final-output terminal mode in `DETACH_ROTATE` without
waiting for optical flow or the horizontal velocity loop. Its defaults in
`attachment_output.yaml` are:

| Parameter | Default |
| --- | --- |
| `detach_terminal_enabled` | true |
| `detach_terminal_start_deg` | 30 degrees from vertical thrust axis |
| `detach_terminal_axis_rate_deg_s` | 10 degrees/s |
| `detach_terminal_yaw_rate_deg_s` | 20 degrees/s |
| `detach_terminal_response_per_s` | 2 /s |

Fresh authorized attitude feedback and actual published history from the same
action are required. The first candidate whose raw desired or previous
published axis is within 30 degrees uses the terminal mode immediately.
Measured tilt does not gate its entrance: waiting for the lagging vehicle to
reach 30 degrees would delay the requested deceleration. The first output is
never projected abruptly into a 30- or 5-degree cone.

For axis error `e` and bounded output interval `dt`, the step is
`min(10 deg/s * dt, e * (1 - exp(-2 /s * dt)))`. This makes the target converge
smoothly to level rather than stop abruptly at 10 degrees/s. Heading remains
separately limited to 20 degrees/s. The target governor can neither cross its
fixed desired axis nor create a counter-tilt command by itself. It is not a
vehicle angular-rate feedback loop and does not guarantee physical attitude.

The latch is committed only after the final validity/authority checks and
successful ROS publication. Temporary loss of the horizontal loop or an actual
tilt excursion cannot restore fast target rotation. After latching, stale
publication history cannot be replaced with a measured seed. The existing
near-level 5-degree mode takes priority when eligible and retains terminal
softening. `RECOVER` continues the true publication history. Action retirement,
ownership or OFFBOARD exit, and successful emergency `ABORT` publication clear
the terminal latch. ABORT preserves its existing immediate leveling behavior.

Thrust support, slew limits, height-continuation logic, normal completion
criteria (5 degrees, 10 degrees/s norm, 0.30 seconds), and the 8-second timeout
are retained. The 3-second parameter describes the nominal trajectory;
terminal shaping can extend the actually published rotation. Horizontal
velocity control retains its existing admission checks and can still request
a corrective tilt after admission. This change neither clips all possible
rearward targets nor validates the previously suspect horizontal estimate.

`output_health.last_wall_command.publication_slew` records `detach_terminal`
rate mode, terminal latch action, softening state, and actual target axis and
heading increments. Replays prove target timing and limits only: vehicle
response, wall/tether forces, and estimator behavior require new measurements.
The first historical run aborted before its replayed terminal completion;
any continuation past that event is explicitly synthetic. Do not interpret
that suffix as evidence of a successful flight.

Build and regression validation must run in Docker `ros1_noetic` as specified
by the workspace instructions. Do not reload the flight process until a fresh
connected/disarmed state is verified. Physical acceptance requires new logs
showing reduced actual reverse tilt and successful normal `RECOVER`, with the
same original stability thresholds.
