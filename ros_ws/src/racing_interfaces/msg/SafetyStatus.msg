std_msgs/Header header

uint32 CLAMP_NONE=0
uint32 CLAMP_TRACK_LIMIT=1
uint32 CLAMP_SPEED=2
uint32 CLAMP_STEERING=4
uint32 CLAMP_ACCELERATION=8
uint32 CLAMP_STEERING_RATE=16
uint32 CLAMP_STALE_INPUT=32
uint32 CLAMP_ESTOP=64

uint8 REASON_NONE=0
uint8 REASON_TRACK_LIMIT=1
uint8 REASON_COMMAND_LIMIT=2
uint8 REASON_STALE_INPUT=3
uint8 REASON_ESTOP_LATCHED=4

string source
builtin_interfaces/Time valid_until
uint32 active_clamps
uint8 reason
uint64 heartbeat
