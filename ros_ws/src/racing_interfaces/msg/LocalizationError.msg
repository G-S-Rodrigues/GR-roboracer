# One ground-truth sample scored against a pose estimate (racing_evaluation).
# header.stamp is the truth sample's time; header.frame_id is truth's frame.
std_msgs/Header header
string source
# The topic whose estimate is scored.
string estimate_topic

# False when no estimate lay within the evaluator's window of this sample:
# the four errors below are then NaN. A stalled estimator is unavailable,
# never zero error.
bool available
# Metres: planar distance, estimate minus truth.
float64 position_error
# Metres, in the track's Frenet frame, estimate minus truth; longitudinal
# wraps at the track length.
float64 lateral_error
float64 longitudinal_error
# Radians, wrapped to [-pi, pi], estimate minus truth.
float64 heading_error

# Running totals since the evaluator's first truth sample.
uint32 sample_count
uint32 available_count
# Metres, over available samples only; NaN while none has been available.
float64 position_rmse
float64 position_maximum
