# Simulation-only direct joint efforts; NEVER publish on vehicle/command.
# A complete command replaces every named motor's effort atomically.
# Motors must have unit gear and finite MJCF ctrlrange limits containing zero.
# Hinge efforts are Nm; slide efforts are N. No speed/position/throttle mode.
builtin_interfaces/Time stamp
# Read from sim/session; changes at startup, pause and reset.
string session_id
# Strictly increasing within a session; duplicate/reordered messages rejected.
uint64 sequence
# Positive simulation-time validity from stamp, bounded by bridge configuration.
builtin_interfaces/Duration valid_for
string[] actuator_names
float64[] efforts
