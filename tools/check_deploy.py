#!/usr/bin/env python3
"""Render Compose templates and check deployment isolation; starts no containers."""

import json
import os
import subprocess
import sys

from check_scaffold import ROOT, closure, packages, require

COMPOSE = ROOT / "deploy" / "compose"
MODES = {
    "workstation-sim": {"simulation", "autonomy", "recorder"},
    "simulator-server": {"simulation", "recorder"},
    "jetson-sim": {"autonomy", "recorder"},
    "kart-live": {"autonomy", "hardware", "recorder"},
}


def rendered(mode, override=False):
    env = os.environ.copy()
    # Known inspection values override the caller's environment; no .env is read.
    for line in (COMPOSE / ".env.example").read_text().splitlines():
        if line and not line.startswith("#"):
            key, value = line.split("=", 1)
            env[key] = value
    env["REPO_ROOT"] = str(ROOT)
    command = ["docker", "compose", "--env-file", str(COMPOSE / ".env.example"),
               "-f", str(COMPOSE / f"{mode}.yaml")]
    if override:
        command += ["-f", str(COMPOSE / "development.override.yaml")]
    command += ["--profile", "*", "config", "--format", "json"]
    result = subprocess.run(command, env=env, capture_output=True, text=True, check=True)
    return json.loads(result.stdout)


def check():
    manifests = packages()
    for mode, expected in MODES.items():
        services = rendered(mode)["services"]
        require(set(services) == expected, f"{mode}: wrong services")
        live = mode == "kart-live"
        require(not ("hardware" in services and "simulation" in services),
                f"{mode}: physical and simulation consumers coexist")
        domains = set()
        for name, service in services.items():
            require(not service.get("devices") and not service.get("privileged"),
                    f"{mode}/{name}: scaffold grants physical device access")
            require(service.get("network_mode") == "host", f"{mode}/{name}: wrong network")
            require(service.get("restart") == "no", f"{mode}/{name}: scaffold auto-restarts")
            require("ALL" in service.get("cap_drop", []), f"{mode}/{name}: capabilities")
            require(service.get("read_only"), f"{mode}/{name}: writable image filesystem")
            domains.add(service["environment"]["ROS_DOMAIN_ID"])
            if name == "recorder":
                require(service["command"] == ["ros2", "bag", "--help"],
                        f"{mode}: recording needs an explicit reviewed configuration")
                require(service.get("profiles") == ["recording"], f"{mode}: recorder default")
            else:
                expected_clock = "use_sim_time:=false" if live else "use_sim_time:=true"
                require(expected_clock in service["command"], f"{mode}/{name}: wrong clock")
                require("namespace:=kart" in service["command"], f"{mode}/{name}: namespace")
                package = {"simulation": "sim_bringup", "autonomy": "autonomy_bringup",
                           "hardware": "hardware_bringup"}[name]
                require(service["command"][:3] == ["ros2", "launch", package],
                        f"{mode}/{name}: wrong launch package")
        require(domains == ({"42"} if live else {"41"}), f"{mode}: example domains differ")

    for mode in ("workstation-sim", "simulator-server"):
        services = rendered(mode, override=True)["services"]
        require(set(services) == MODES[mode], f"{mode}: development override changed services")
        mounts = services["simulation"]["volumes"]
        require(len(mounts) == 1 and mounts[0]["read_only"]
                and mounts[0]["target"] == "/workspace",
                f"{mode}: development mount must remain explicit and read-only")

    targets = {"simulation": "sim_bringup", "autonomy": "autonomy_bringup",
               "hardware": "hardware_bringup", "recorder": "interfaces"}
    for image, target in targets.items():
        copied = set()
        dockerfile = (ROOT / "deploy" / "docker" / f"{image}.Dockerfile").read_text()
        for line in dockerfile.splitlines():
            if line.startswith("COPY src/"):
                source = ROOT / line.split()[1]
                require(source.is_dir(), f"{image}: missing COPY source")
                copied.update(name for name, (path, _) in manifests.items()
                              if path == source or source in path.parents)
        require(copied == closure(target, manifests),
                f"{image}: copied packages differ from its dependency closure")
        if image == "simulation":
            require("COPY simulation/ /opt/kart/simulation/" in dockerfile and
                    "pip install --no-cache-dir /opt/kart/simulation" in dockerfile,
                    "Simulation image must install the standalone Python core")
        else:
            require("COPY simulation/" not in dockerfile,
                    f"{image}: unexpected standalone simulator source")
    print("Deployment templates OK: four modes, two development overlays, "
          "clock selection, no device access, and four image dependency closures.")


if __name__ == "__main__":
    try:
        check()
    except (ValueError, OSError, KeyError, subprocess.CalledProcessError) as error:
        print(f"Deployment check failed: {error}", file=sys.stderr)
        if isinstance(error, subprocess.CalledProcessError):
            print(error.stderr, file=sys.stderr)
        sys.exit(1)
