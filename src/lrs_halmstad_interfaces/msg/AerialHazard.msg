# Standard metric detection. The Detection3D header is the acquisition
# timestamp and its frame is the frame in which the bbox and pose are stated.
vision_msgs/Detection3D detection

# UAVs that contributed evidence to this track.
string[] source_uavs

# Track-level fusion state.
uint8 TENTATIVE=0
uint8 CONFIRMED=1
uint8 CONFLICT=2
uint8 state

# Track history.
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen

# Maximum permitted persistence after last_seen.
builtin_interfaces/Duration ttl

# Combined support quality in [0, 1].
float32 support_quality

# Short machine-readable provenance.
string provenance
