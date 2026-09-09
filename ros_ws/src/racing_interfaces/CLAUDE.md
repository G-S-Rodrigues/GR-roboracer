# racing_interfaces

This package owns the racing-domain ROS interface contract.

- Prefer standard ROS messages for sensors and commands. Add custom interfaces only for racing-domain data.
- Every custom message carries `std_msgs/Header`.
- Add `source` and `valid_until` when consumers need to reject stale data.
- Treat field names, units, constants, and types as stack-wide API: update every producer, consumer, and contract test together.
- Keep simulator-specific types and names out of this package.
