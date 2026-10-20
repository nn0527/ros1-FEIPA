# Parameter transaction state, NOT evidence of actual EKF measurement fusion.
std_msgs/Header header
bool enabled
string phase
string episode_id
bool snapshot_valid
bool restore_required
bool parameters_restored
# Original parameters acknowledged, application buffer elapsed, no unresolved transaction.
# Consumers must additionally require fresh post-ready sensor/estimator samples.
bool control_ready
int32 original_flow
int32 original_range
int32 target_flow
int32 target_range
# -1 means unknown; values are only confirmed PARAM_SET/PARAM_PULL results.
int32 confirmed_flow
int32 confirmed_range
time request_stamp
time confirmed_stamp
time ready_after
float32 measured_tilt_deg
uint32 retry_count
string reason
