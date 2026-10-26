std_msgs/Header header

uint8 MODE_MANUAL=0
uint8 MODE_STANDBY=1
uint8 MODE_CEILING=2
uint8 MODE_WALL=3
uint8 MODE_ABORT=4

uint8 mode
bool enable_control
string source
string reason
# Nonzero only after a fresh CH12 edge in captured wall RECOVER/EXIT.
# The event time binds consent to the current departure, not the next cycle.
time pilot_handover_request_time
