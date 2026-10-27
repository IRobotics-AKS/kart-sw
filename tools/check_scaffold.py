#!/usr/bin/env python3
"""Check repository structure and dependency isolation without requiring ROS."""

import ast
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = {
    "interfaces": "common",
    "description": "common",
    "sim_ros": "simulation",
    "perception": "autonomy",
    "localization": "autonomy",
    "planning": "autonomy",
    "control": "autonomy",
    "sensor_drivers": "hardware",
    "vehicle_bridge": "hardware",
    "vehicle_manager": "system",
    "autonomy_bringup": "bringup",
    "sim_bringup": "bringup",
    "hardware_bringup": "bringup",
}
DEPENDENCY_TAGS = {
    "depend", "build_depend", "build_export_depend", "exec_depend",
    "buildtool_depend", "buildtool_export_depend",
}
EXTERNAL_DEPENDENCIES = {
    "ament_cmake", "ament_python", "launch", "launch_ros", "nav_msgs", "rclpy",
    "robot_state_publisher", "rosgraph_msgs", "rosidl_default_generators",
    "rosidl_default_runtime", "sensor_msgs",
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def packages():
    result = {}
    for path in sorted((ROOT / "src").rglob("package.xml")):
        manifest = ET.parse(path).getroot()
        name = manifest.findtext("name")
        require(name not in result, f"Duplicate package: {name}")
        result[name] = (path.parent, manifest)
    return result


def closure(name, manifests):
    found = set()
    pending = [name]
    while pending:
        current = pending.pop()
        if current in found:
            continue
        found.add(current)
        manifest = manifests[current][1]
        pending.extend(
            child.text for child in manifest
            if child.tag in DEPENDENCY_TAGS and child.text in manifests
        )
    return found


def imports(tree):
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            yield from (alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            yield node.module


def check():
    manifests = packages()
    require(set(manifests) == set(EXPECTED), "ROS package set differs from the design")
    for name, (path, manifest) in manifests.items():
        require(path == ROOT / "src" / EXPECTED[name] / name,
                f"{name}: incorrect package ownership")
        require(manifest.findtext("version") == "0.0.0",
                f"{name}: update scaffold status when introducing a version")
        for child in manifest:
            if child.tag in DEPENDENCY_TAGS:
                require(child.text in manifests or child.text in EXTERNAL_DEPENDENCIES,
                        f"{name}: unknown dependency {child.text}; declare external "
                        "dependencies explicitly")
        build_type = manifest.findtext("export/build_type")
        if build_type == "ament_python":
            for item in ("setup.py", "setup.cfg", f"resource/{name}", f"{name}/__init__.py"):
                require((path / item).is_file(), f"{name}: missing {item}")
        else:
            require(build_type == "ament_cmake", f"{name}: unexpected build type")
            cmake = (path / "CMakeLists.txt").read_text()
            require(f"project({name})" in cmake and "ament_package()" in cmake,
                    f"{name}: missing ament installation metadata")

    allowed = {
        "autonomy_bringup": {
            "autonomy_bringup", "interfaces", "description",
            "perception", "localization", "planning",
            "control", "vehicle_manager",
        },
        "sim_bringup": {"sim_bringup", "interfaces", "sim_ros"},
        "hardware_bringup": {
            "hardware_bringup", "interfaces",
            "sensor_drivers", "vehicle_bridge",
        },
    }
    for target, expected in allowed.items():
        actual = closure(target, manifests)
        require(actual == expected, f"{target}: dependency boundary changed: {actual}")

    checked = 0
    for base in ("src", "simulation/src", "tools"):
        require((ROOT / base).is_dir(), f"Missing source directory: {base}")
        for path in sorted((ROOT / base).rglob("*.py")):
            if any(part in {"build", "install", "__pycache__"} or part.endswith(".egg-info")
                   for part in path.relative_to(ROOT).parts):
                continue
            tree = ast.parse(path.read_text(), filename=str(path))
            modules = {name.split(".")[0] for name in imports(tree)}
            relative = path.relative_to(ROOT)
            if relative.parts[0] == "simulation":
                forbidden = {"rclpy", "rclcpp", "sensor_msgs", "nav_msgs", "interfaces",
                             "vehicle_bridge", "depthai", "serial", "can"}
            elif relative.parts[:2] in {("src", "autonomy"), ("src", "system")}:
                forbidden = {"mujoco", "sim", "sim_ros", "vehicle_bridge",
                             "sensor_drivers", "depthai", "serial", "can"}
                if "algorithms" in relative.parts:
                    forbidden |= {"rclpy", "interfaces", "nav_msgs", "sensor_msgs"}
            else:
                forbidden = set()
            require(not modules & forbidden,
                    f"{relative}: forbidden imports {modules & forbidden}")
            checked += 1

    for directory in ("simulation", "firmware", "handoff"):
        require((ROOT / directory / "COLCON_IGNORE").is_file(),
                f"{directory}: missing colcon exclusion")
    print(f"Scaffold OK: {len(manifests)} ROS packages; {checked} Python files; "
          "simulation/autonomy/hardware dependency boundaries intact.")


if __name__ == "__main__":
    try:
        check()
    except (ValueError, SyntaxError, ET.ParseError, OSError) as error:
        print(f"Scaffold check failed: {error}", file=sys.stderr)
        sys.exit(1)
