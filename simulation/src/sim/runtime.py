"""ROS-independent MuJoCo runtime with explicit simulation joint efforts."""

from dataclasses import dataclass
from importlib.resources import files
import math
from pathlib import Path
import random

import mujoco


@dataclass(frozen=True)
class Snapshot:
    time_s: float
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_mps: tuple[float, float, float]
    angular_velocity_rps: tuple[float, float, float]
    acceleration_mps2: tuple[float, float, float]
    gyro_rps: tuple[float, float, float]


class Simulation:
    """Load MJCF; publishable snapshots use body pose and co-located IMU sensors.

    Pose is the body origin in MuJoCo world; twist is at that origin in body
    axes. IMU vectors use the sensor site's axes. Orientation is truth only.
    Noise is independent Gaussian noise per axis/sample, not a BNO055 model.
    """

    def __init__(self, model_path: str, *, body: str = "base_link",
                 accelerometer: str = "imu_accel", gyro: str = "imu_gyro",
                 accel_noise_std: float, gyro_noise_std: float, seed: int,
                 timestep: float | None = None):
        for name, value in (("accel_noise_std", accel_noise_std),
                            ("gyro_noise_std", gyro_noise_std)):
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
            raise ValueError("seed must be a nonnegative integer")
        if model_path == "demo":
            model_path = str(files("sim").joinpath("assets/bridge_demo.xml"))
        if not model_path or not Path(model_path).is_file():
            raise ValueError("model must name an MJCF file, or explicitly select 'demo'")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        if timestep is not None:
            if not math.isfinite(timestep) or timestep <= 0:
                raise ValueError("timestep must be finite and positive")
            self.model.opt.timestep = timestep
        if not math.isfinite(self.model.opt.timestep) or self.model.opt.timestep < 1e-9:
            raise ValueError("model timestep must be finite and at least one nanosecond")
        self.actuators = self._actuators()
        self._body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
        if self._body_id <= 0:
            raise ValueError(f"missing non-world body: {body}")
        self._accel_id = self._sensor(accelerometer, mujoco.mjtSensor.mjSENS_ACCELEROMETER)
        self._gyro_id = self._sensor(gyro, mujoco.mjtSensor.mjSENS_GYRO)
        site = self.model.sensor_objid[self._accel_id]
        if site != self.model.sensor_objid[self._gyro_id]:
            raise ValueError("accelerometer and gyro must use the same site")
        if self.model.site_bodyid[site] != self._body_id:
            raise ValueError("IMU site must belong to the configured body")
        self.data = mujoco.MjData(self.model)
        self.accel_noise_std = accel_noise_std
        self.gyro_noise_std = gyro_noise_std
        self._seed = seed
        self.reset()

    def _actuators(self):
        """Accept only unit-gear joint motors with effort limits containing zero.

        Motor input is joint force (N, slide) or torque (Nm, hinge). Reject
        servos, tendons, free/ball joints and activation dynamics rather than
        quietly interpreting their controls as forces.
        """
        result = {}
        for index in range(self.model.nu):
            model = self.model
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
            joint = model.actuator_trnid[index, 0]
            low, high = map(float, model.actuator_ctrlrange[index])
            valid = (
                name and model.actuator_trntype[index] == mujoco.mjtTrn.mjTRN_JOINT
                and model.jnt_type[joint] in (int(mujoco.mjtJoint.mjJNT_SLIDE), int(mujoco.mjtJoint.mjJNT_HINGE))
                and model.actuator_dyntype[index] == mujoco.mjtDyn.mjDYN_NONE
                and model.actuator_gaintype[index] == mujoco.mjtGain.mjGAIN_FIXED
                and model.actuator_biastype[index] == mujoco.mjtBias.mjBIAS_NONE
                and model.actuator_gainprm[index, 0] == 1.0
                and tuple(model.actuator_gear[index]) == (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                and model.actuator_ctrllimited[index]
                and math.isfinite(low) and math.isfinite(high) and low <= 0 <= high and low < high
            )
            if not valid:
                raise ValueError(f"actuator {name or index} must be a named, bounded, unit-gear hinge/slide motor")
            unit = "N" if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_SLIDE else "Nm"
            result[name] = (index, low, high, unit)
        return result

    def set_efforts(self, names, efforts):
        """Atomically replace all efforts; incomplete/duplicate commands fail."""
        if len(names) != len(efforts) or len(set(names)) != len(names) or set(names) != set(self.actuators):
            raise ValueError("command must include every actuator exactly once")
        controls = []
        for name, effort in zip(names, efforts):
            index, low, high, _ = self.actuators[name]
            if not math.isfinite(effort) or not low <= effort <= high:
                raise ValueError(f"effort for {name} must be finite and within [{low}, {high}]")
            controls.append((index, effort))
        for index, effort in controls:
            self.data.ctrl[index] = effort

    def clear_efforts(self):
        self.data.ctrl[:] = 0.0

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    def _sensor(self, name, expected_type):
        index = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if (index < 0 or self.model.sensor_type[index] != expected_type
                or self.model.sensor_dim[index] != 3):
            raise ValueError(f"missing or wrong sensor type: {name}")
        return index

    def _measurement(self, index, std):
        address = self.model.sensor_adr[index]
        return tuple(float(x) + self._rng.gauss(0, std)
                     for x in self.data.sensordata[address:address + 3])

    def _snapshot(self):
        # mj_step integrates qpos after evaluating sensors. Re-evaluate at the
        # new state so pose, velocity, sensors and timestamp all describe it.
        mujoco.mj_forward(self.model, self.data)
        velocity = self.data.cvel[self._body_id].copy()
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY,
                                self._body_id, velocity, 1)
        q = self.data.xquat[self._body_id]
        sample = Snapshot(
            float(self.data.time), tuple(map(float, self.data.xpos[self._body_id])),
            tuple(float(q[i]) for i in (1, 2, 3, 0)),
            tuple(map(float, velocity[3:])), tuple(map(float, velocity[:3])),
            self._measurement(self._accel_id, self.accel_noise_std),
            self._measurement(self._gyro_id, self.gyro_noise_std))
        values = (sample.time_s, *sample.position_m, *sample.orientation_xyzw,
                  *sample.linear_velocity_mps, *sample.angular_velocity_rps,
                  *sample.acceleration_mps2, *sample.gyro_rps)
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("MuJoCo produced a nonfinite snapshot")
        return sample

    def step(self) -> Snapshot:
        previous = self.data.time
        mujoco.mj_step(self.model, self.data)
        if self.data.time <= previous:
            raise RuntimeError("MuJoCo time stopped or reset unexpectedly")
        self.snapshot = self._snapshot()
        return self.snapshot

    def reset(self) -> Snapshot:
        mujoco.mj_resetData(self.model, self.data)
        self._rng = random.Random(self._seed)
        self.snapshot = self._snapshot()
        return self.snapshot

    def open_viewer(self):
        """Caller owns the passive viewer's sync/close lifecycle."""
        import mujoco.viewer
        return mujoco.viewer.launch_passive(self.model, self.data)
