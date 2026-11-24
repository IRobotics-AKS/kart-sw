# Simulation runtime configuration

`bridge_demo.yaml` is runnable with `model:=demo`. Its noise and command timeouts
are synthetic test settings, not measured BNO055 or kart limits. Copy and review
these parameters for a custom MJCF. The model owns timestep and motor effort
limits; launch may explicitly override timestep. See the bridge runbook for
frames, units, timestamp, QoS and reset contracts.
