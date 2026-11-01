# The simulator's own lap and collision counters, as the sim reports them.
#
# DEBUG AND CROSS-CHECK ONLY. The league restricts lap and collision counters
# at race time, so no stack node may subscribe to this topic: only tests and
# tools read it (#44 part 10). Published by racing_sim_autodrive per frame.
std_msgs/Header header
uint32 lap_count
float64 lap_time
float64 last_lap_time   # inf before the first lap
float64 best_lap_time   # inf before the first lap
uint32 collisions
