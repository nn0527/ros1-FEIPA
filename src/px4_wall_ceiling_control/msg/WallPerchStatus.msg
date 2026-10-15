std_msgs/Header header

uint8 STATE_IDLE=0
uint8 STATE_FRONT_WALL_DETECT=1
uint8 STATE_STABILIZE_HOVER=2
uint8 STATE_SLOW_APPROACH=3
uint8 STATE_FLIP_TO_WALL=4
uint8 STATE_WALL_CAPTURE=5
uint8 STATE_WALL_HOLD=6
uint8 STATE_WALL_PIN=7
uint8 STATE_DETACH_ROTATE=8
uint8 STATE_RECOVER=9
uint8 STATE_EXIT=10
uint8 STATE_ABORT=11

uint8 state
string state_name
bool enabled
bool acquire_request
bool keep_ownership
bool candidate_valid
bool front_range_fresh
bool top_range_fresh
bool sensor_health_fresh
bool top_preflight_ready
bool odom_fresh
bool flight_state_fresh
bool front_ready
bool flip_ready
bool top_contact_ready
bool top_clear_ready
bool fault_detected
bool neutral_setpoint_requested
bool attitude_setpoint_valid
float32 front_distance_raw_m
float32 front_distance_filtered_m
float32 top_distance_raw_m
float32 top_distance_filtered_m
float32 progress
geometry_msgs/Quaternion attitude_setpoint
# MAVROS AttitudeTarget convention: positive normalized collective thrust.
float32 thrust_normalized
uint32 fault_count
string reason
# Entry height uses the frozen ground reference from before arming (ENU up).
bool altitude_reference_valid
float32 height_above_takeoff_m
float32 min_entry_height_m
bool entry_height_ready
# Per-maneuver milestones survive abort/release; reset on the next wall acquire.
bool hold_achieved
string reached_state_name
bool detach_ready
bool detach_completed
bool recovery_completed
bool detach_pending
string detach_block_reason

# Automatic pre-flip approach mode and live sensor pause state.
bool auto_approach_enabled
bool approach_paused

# Optional optical-flow-aided horizontal speed loop, using PX4 fused ENU velocity.
bool horizontal_velocity_enabled
bool horizontal_velocity_ready
bool horizontal_velocity_active
bool horizontal_velocity_stable
string horizontal_velocity_reason
float32 horizontal_velocity_x_enu_mps
float32 horizontal_velocity_y_enu_mps
float32 horizontal_target_x_enu_mps
float32 horizontal_target_y_enu_mps
float32 horizontal_accel_x_enu_mps2
float32 horizontal_accel_y_enu_mps2
uint8 optical_flow_quality

# Vertical recovery/approach loop diagnostics; local ENU height, not AGL.
bool vertical_control_active
bool vertical_reference_valid
float32 vertical_reference_m
string vertical_reference_source
float32 vertical_target_enu_mps
float32 vertical_thrust_correction

# Stable POSCTL feedforward and horizontal feedback through the handshake.
bool horizontal_equilibrium_enabled
bool horizontal_equilibrium_valid
bool horizontal_equilibrium_frozen
string horizontal_equilibrium_reason
geometry_msgs/Quaternion horizontal_equilibrium_attitude
bool horizontal_prestream_valid
geometry_msgs/Quaternion horizontal_prestream_attitude
