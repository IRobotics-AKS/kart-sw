# Integration checks

The scaffold checker verifies package/dependency boundaries and Python syntax.
The deployment checker renders Compose and checks service isolation, clocks,
device access, and restart policy. Neither establishes runtime readiness.

With runtime implementations, add workstation closed-loop tests using estimated
state; truth-only scoring; reset/time jumps; expiry; disconnect/reconnect;
critical-child failure; session invalidation; and no automatic rearming. Add
cross-host wall-clock timing tests on the Jetson separately from simulation time.
