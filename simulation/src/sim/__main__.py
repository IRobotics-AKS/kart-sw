"""Small standalone viewer/headless smoke runner, independent of ROS."""
import argparse
import math
import time

from sim.runtime import Simulation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, help="MJCF path or 'demo'")
    parser.add_argument("--duration", type=float, default=5.0, help="simulation seconds")
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("duration must be finite and positive")
    # This CLI is an explicitly ideal, sensor-only smoke test.
    sim = Simulation(args.model, accel_noise_std=0.0, gyro_noise_std=0.0, seed=0)
    viewer = sim.open_viewer() if args.viewer else None
    started = time.monotonic()
    try:
        while sim.snapshot.time_s < args.duration:
            sim.step()
            if viewer is not None:
                if not viewer.is_running():
                    break
                viewer.sync()
                time.sleep(max(0.0, started + sim.snapshot.time_s - time.monotonic()))
    finally:
        if viewer is not None:
            viewer.close()
    print(f"Simulated {sim.snapshot.time_s:.3f}s in {time.monotonic() - started:.3f}s wall time")
    print(f"Body position (m): {sim.snapshot.position_m}")


if __name__ == "__main__":
    main()
