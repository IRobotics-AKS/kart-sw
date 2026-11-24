"""Open the Dreamer kart model in the MuJoCo viewer.

The model lives in simulation/models/dreamer/kart.xml (meshes in its assets/ folder),
so there is one copy of it. Run from the repository root:

    .venv/bin/python simulation/src/sim/simulator.py
"""
import pathlib

import mujoco
import mujoco.viewer

MODEL = pathlib.Path(__file__).resolve().parents[2] / "models" / "dreamer" / "kart.xml"


def load(path=MODEL):
    """(model, data) for the kart.xml at path."""
    model = mujoco.MjModel.from_xml_path(str(path))
    return model, mujoco.MjData(model)


def main():
    model, data = load()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()
