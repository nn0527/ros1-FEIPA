std_msgs/Header header

bool ceiling_valid
bool front_valid

# Age of the last valid sample as measured at this node. Infinity means that
# no valid sample has been received since startup.
float32 ceiling_age_s
float32 front_age_s

# Monotonic counters allow consumers and tests to distinguish real samples
# from repeated status publications.
uint32 ceiling_sequence
uint32 front_sequence
uint32 ceiling_invalid_count
uint32 front_invalid_count

# 0=valid; positive=L1 device error (e.g. 255); negative=local input error.
# -1 timeout, -2 no data, -3 nonfinite, -4 out of range, -5 parse,
# -6 configuration, -7 driver dependency, -8 serial, -9 diagnostic format.
int32 ceiling_error_code
int32 front_error_code
string ceiling_error_type
string front_error_type
string ceiling_error_reason
string front_error_reason
