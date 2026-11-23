"""Run the real ROS 2 Humble and Gazebo Fortress checks on the demo package.

Run from any folder: python scripts/ros_gates.py

It uses an existing billee-software-2027 dev container image and its pixi environment.
The software repo is mounted read-only and the container has no network, so the checks
cannot change the repo or reach a rover on the local network.

Optional environment variables:
  BILLEE_SOFTWARE_DIR  Path to billee-software-2027 (default: ~/billee-software-2027)
  BILLEE_ROS_IMAGE     Docker image (default: newest vsc-billee-software-2027 image)
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from common import ROOT, IsolatedProfile, find_blender, run

MOUNT = "/workspaces/billee-software-2027"


def find_image() -> str:
    configured = os.environ.get("BILLEE_ROS_IMAGE")
    if configured:
        return configured
    listed = run(["docker", "images", "--format", "{{.Repository}}:{{.Tag}}"]).stdout.splitlines()
    images = [i for i in listed if i.startswith("vsc-billee-software-2027") and "-uid:" not in i]
    if not images:
        sys.exit("No billee-software-2027 dev container image found. Open that repo's dev container once.")
    return images[0]


def main() -> int:
    software = Path(os.environ.get("BILLEE_SOFTWARE_DIR", Path.home() / "billee-software-2027")).resolve()
    if not (software / "ros2_ws" / ".pixi" / "envs" / "default" / "bin" / "ros2").exists():
        sys.exit(f"No installed pixi ROS environment in {software}/ros2_ws.")
    image = find_image()
    evidence = ROOT / "dist" / "ros-gates"
    shutil.rmtree(evidence, ignore_errors=True)
    evidence.mkdir(parents=True)

    with tempfile.TemporaryDirectory(prefix="rab-ros-") as folder:
        with IsolatedProfile() as env:
            made = run([find_blender(), "--background", "--factory-startup", "--python-exit-code", "1",
                        "--python", str(ROOT / "examples" / "generate_demo.py"), "--", folder], env)
        if made.returncode != 0:
            sys.exit("Demo generation failed:\n" + made.stdout[-3000:] + made.stderr[-1000:])
        package = next(p for p in Path(folder).iterdir() if p.is_dir())
        os.chmod(folder, 0o755)
        command = [
            "docker", "run", "--rm", "--network", "none", "--user", f"{os.getuid()}:{os.getgid()}",
            "--entrypoint", "bash",
            "-v", f"{software}:{MOUNT}:ro",
            "-v", f"{ROOT}:/toolkit:ro",
            "-v", f"{folder}:/demo:ro",
            "-v", f"{evidence}:/evidence",
            image, "/toolkit/tests/ros/run_gates.sh", f"/demo/{package.name}", "/toolkit", "/evidence",
        ]
        process = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 encoding="utf-8", errors="replace", timeout=600, check=False)
    (evidence / "summary.log").write_text(process.stdout, encoding="utf-8")
    print(process.stdout)
    print("MoveIt was not run: it is not installed in the BILLEE Humble environment.")
    return process.returncode


if __name__ == "__main__":
    sys.exit(main())
