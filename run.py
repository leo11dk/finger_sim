"""Run the finger inside the stock MuJoCo viewer, with no browser or server."""
import argparse
import os
from pathlib import Path
import shutil
import sys
import sysconfig
import time

import mujoco
import numpy as np
from experiment import FingerExperiment
from viewer_controls import ViewerControls
from visuals import draw_springs, draw_reel_cables

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "scenes/starter.xml")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps", type=int, default=1000)
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--steps must be positive")
    if not args.headless:
        from mujoco import viewer as mj_viewer
        if sys.platform == "darwin" and getattr(mj_viewer, "_MJPYTHON", None) is None:
            launcher = Path(sysconfig.get_path("scripts")) / "mjpython"
            if not launcher.is_file():
                candidate = shutil.which("mjpython")
                if candidate is None:
                    parser.error("mjpython not found; install requirements.txt")
                launcher = Path(candidate)
            os.execv(sys.executable, [sys.executable, str(launcher), str(Path(__file__).resolve()), *sys.argv[1:]])
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    print(f"MuJoCo {mujoco.__version__} | {args.model.name} | nq={model.nq}, nv={model.nv}, nu={model.nu}", flush=True)
    if args.headless:
        for _ in range(args.steps):
            mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError("Simulation produced non-finite state")
        if any(w.number for w in data.warning):
            raise RuntimeError("MuJoCo reported simulation warnings")
        print(f"Passed {args.steps} steps; simulated time {data.time:.3f} s")
        return
    try:
        controls = ViewerControls(FingerExperiment(model, data))
    except KeyError:
        controls = None
    callback = controls.key_callback if controls else None
    with mj_viewer.launch_passive(model, data, key_callback=callback,
                                 show_left_ui=True, show_right_ui=True) as viewer:
        print("Native Control sliders: reel torque, spring 1/2 stiffness, pulley 1/2 radius. Home resets; End restores defaults; Space pauses.", flush=True)
        try:
            while viewer.is_running():
                start = time.monotonic()
                with viewer.lock():
                    if controls:
                        controls.process()
                    if controls is None or not controls.paused:
                        mujoco.mj_step(model, data)
                    mujoco.mj_forward(model, data)
                    viewer.user_scn.ngeom = 0
                    draw_springs(model, data, viewer.user_scn)
                    draw_reel_cables(model, data, viewer.user_scn)
                viewer.sync()
                time.sleep(max(0., model.opt.timestep-(time.monotonic()-start)))
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
