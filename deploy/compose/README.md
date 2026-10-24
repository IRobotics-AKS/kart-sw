# Host deployment templates

| File | Host | Default services |
| --- | --- | --- |
| workstation-sim.yaml | Workstation | simulation, autonomy |
| simulator-server.yaml | Workstation | simulation |
| jetson-sim.yaml | Jetson | autonomy |
| kart-live.yaml | Jetson | autonomy, hardware |

Each also declares an optional `recording` profile whose current command is
`ros2 bag --help`. Configure topics, simulation clock use, bounded storage, and
a persistent destination before enabling real recording. Nothing records by default.

Inspect templates without starting containers:

```sh
docker compose --env-file deploy/compose/.env.example \
  -f deploy/compose/workstation-sim.yaml config
python3 tools/check_deploy.py
```

Copy the example environment file to an ignored local `.env` for your host.
Use the same simulation domain/source/configuration on both hosts during remote
simulation, and a different domain for live operation. The example domain values
are illustrative discovery settings; they do not authorize or prevent actuation.
Middleware transport/interface/firewall settings remain to be validated.

Native Linux host networking is the target. No services have device mappings,
GPU access, privileged mode, host IPC, automatic restart, or startup arming.
The 10-second shutdown grace period is an unvalidated process-management default,
not a command validity, motor watchdog, or stopping budget.

These files are **not runnable vehicle deployments**: launch guards report missing
runtime implementation. The live/simulation operational mode lock, readiness,
session invalidation, hardware mapping, calibration, and physical stop validation
must be implemented before live deployment. Separate Compose project names do not
exclude one another or stop an already running live stack.

`development.override.yaml` only mounts read-only source under /workspace for
simulation development; it never silently rebuilds or overrides installed code.
