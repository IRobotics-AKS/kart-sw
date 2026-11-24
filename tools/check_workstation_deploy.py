#!/usr/bin/env python3
"""Check portable local Compose configuration without starting containers."""
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def check():
    env = {key: value for key, value in os.environ.items() if not key.startswith('KART_SIM_')}
    with tempfile.TemporaryDirectory(prefix='kart compose check ') as directory:
        # An unrelated local .env must not leak into the wrapper's configuration.
        Path(directory, '.env').write_text('KART_SIM_MODEL=unexpected\nCOMPOSE_FILE=missing.yaml\n')
        output = subprocess.check_output(
            [str(ROOT / 'tools/sim-docker'), 'config', '--format', 'json'],
            cwd=directory, env=env, text=True)
    config = json.loads(output)
    assert set(config['services']) == {'simulation'}, 'Only the simulator belongs in local mode'
    service = config['services']['simulation']
    assert Path(service['build']['context']).resolve() == ROOT, 'Build context must resolve to this checkout'
    assert service['build']['dockerfile'] == 'deploy/docker/simulation.Dockerfile'
    assert 'platform' not in service, 'Do not force Intel emulation on Apple Silicon'
    assert service['network_mode'] == 'none', 'Workstation ROS must stay local'
    assert not any(service.get(key) for key in ('ports', 'volumes', 'devices', 'privileged', 'gpus'))
    assert service['read_only'] and service['restart'] == 'no'
    assert service['cap_drop'] == ['ALL']
    assert service['environment']['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    assert 'use_sim_time:=true' in service['command']
    assert 'model:=demo' in service['command'] and 'rendering:=false' in service['command']
    assert service['healthcheck']['test'][-1] == '--health'
    recipe = (ROOT / 'deploy/docker/simulation.Dockerfile').read_text()
    assert 'ARG ROS_BASE_IMAGE=ros:jazzy-ros-base-noble@sha256:' in recipe
    assert 'COPY src/hardware' not in recipe and 'COPY src/autonomy' not in recipe
    print('Workstation Compose OK: portable context, native architecture, local ROS, simulation only.')


if __name__ == '__main__':
    check()
