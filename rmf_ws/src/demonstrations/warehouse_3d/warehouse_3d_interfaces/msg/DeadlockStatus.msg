# DeadlockStatus.msg
# Status reported when conflicting robot paths are exposed to external logic

bool deadlock_detected
string[] involved_robots
string[] conflict_locations
string suggested_resolution
builtin_interfaces/Time timestamp
