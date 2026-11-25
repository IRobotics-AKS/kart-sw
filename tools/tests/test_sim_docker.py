"""Check wrapper path/argument handling using a fake Docker CLI; no daemon."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class WrapperTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='sim wrapper ')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'checkout with spaces'
        (self.root / 'tools').mkdir(parents=True)
        shutil.copy2(ROOT / 'tools/sim-docker', self.root / 'tools/sim-docker')
        self.log = Path(self.temp.name) / 'calls.jsonl'
        binary = Path(self.temp.name) / 'bin'
        binary.mkdir()
        docker = binary / 'docker'
        docker.write_text('''#!/usr/bin/env python3
import json, os, sys
with open(os.environ['DOCKER_TEST_LOG'], 'a') as stream:
    stream.write(json.dumps(sys.argv[1:]) + '\\n')
if sys.argv[1:] == ['info'] and os.environ.get('DOCKER_TEST_DOWN'):
    sys.exit(1)
''')
        docker.chmod(0o755)
        self.env = dict(os.environ, PATH=str(binary) + os.pathsep + os.environ['PATH'],
                        DOCKER_TEST_LOG=str(self.log))
        for key in list(self.env):
            if key.startswith('KART_SIM_'):
                del self.env[key]

    def run_wrapper(self, *args, **env):
        return subprocess.run([str(self.root / 'tools/sim-docker'), *args], cwd='/tmp',
                              env={**self.env, **env}, capture_output=True, text=True)

    def calls(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()]

    def test_external_cwd_and_space_in_path(self):
        result = self.run_wrapper('config')
        self.assertEqual(result.returncode, 0, result.stderr)
        call = self.calls()[-1]
        self.assertIn(str(self.root.resolve() / 'deploy/compose/workstation-local.yaml'), call)
        self.assertIn('/dev/null', call)
        self.assertNotIn(['info'], self.calls())

    def test_exec_preserves_arguments_without_evaluation(self):
        literal = 'spaces; $(do-not-run)'
        result = self.run_wrapper('exec', 'printf', '%s', literal)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls()[-1][-5:], ['simulation', 'kart-entrypoint', 'printf', '%s', literal])

    def test_daemon_failure_does_not_start_service(self):
        result = self.run_wrapper('up', DOCKER_TEST_DOWN='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Docker is not reachable', result.stderr)
        self.assertFalse(any('up' in call for call in self.calls()))

    def test_reset_pauses_and_leaves_paused(self):
        result = self.run_wrapper('reset')
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self.calls()
        self.assertIn('/kart/sim/pause', calls[-2])
        self.assertIn('{data: true}', calls[-2])
        self.assertIn('/kart/sim/reset', calls[-1])
        self.assertFalse(any('{data: false}' in call for call in calls))


if __name__ == '__main__':
    unittest.main()
