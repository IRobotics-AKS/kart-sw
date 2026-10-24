# Deployment scaffold

Docker recipes, separate per-host Compose templates, and version/release worksheets
are present. They describe package installation and service ownership; they do not
implement a runtime, a validated image, hardware integration, or operational mode
locking. See docker/README.md and compose/README.md for inspection/build instructions.
Keep firmware outside images and keep runtime secrets outside this repository.
