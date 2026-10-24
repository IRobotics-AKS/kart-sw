# Developer checks

Run from any directory with Python 3.9+ (runtime packages target Python 3.12):

```sh
python3 tools/check_scaffold.py
python3 tools/check_deploy.py
```

The first uses only the standard library. The second requires Docker Compose v2+
and renders configuration without contacting a Docker daemon or starting services.
It uses the checked-in example environment, not local deployment credentials.

These are scaffold/dependency checks, not runtime or physical safety tests.
