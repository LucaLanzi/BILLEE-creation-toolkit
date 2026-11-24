"""Prove MoveIt plans and executes the exported arm on the unmodified BILLEE rover stack.

Run from any folder: python scripts/moveit_check.py

The packages are exported through the Blender operators. The check needs a MoveIt layer made by
scripts/setup_moveit_env.sh. That layer sits on a read-only view of billee-software-2027's
environment, the repo is mounted read-only, and the container has no network.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from common import ROOT, IsolatedProfile, find_blender, run
from ros_gates import MOUNT, find_image


def strip_render_sensor(package: Path) -> None:
    """This check has no display, so only temporary copies drop the ZED render sensor."""
    path = package / "urdf" / "robot_gz.urdf.xacro"
    text = path.read_text(encoding="utf-8")
    text = re.sub(r'<sensor name="zed2i".*?</sensor>', "", text, flags=re.S)
    text = re.sub(r'<plugin name="ignition::gazebo::systems::Sensors".*?</plugin>', "", text, flags=re.S)
    path.write_text(text, encoding="utf-8")


def main() -> int:
    software = Path(os.environ.get("BILLEE_SOFTWARE_DIR", Path.home() / "billee-software-2027")).resolve()
    default_layer = Path.home() / ".cache" / "billee-creation-toolkit" / "moveit-env"
    layer = Path(os.environ.get("BILLEE_MOVEIT_ENV", default_layer)).resolve()
    if not (layer / "upper" / "conda-meta").is_dir():
        sys.exit(f"No MoveIt layer in {layer}. Run: sh scripts/setup_moveit_env.sh")
    evidence = ROOT / "dist" / "moveit-check"
    shutil.rmtree(evidence, ignore_errors=True)
    evidence.mkdir(parents=True)
    lower = software / "ros2_ws" / ".pixi" / "envs" / "default"
    overlay = (f"type=volume,dst={MOUNT}/ros2_ws/.pixi/envs/default,volume-driver=local,volume-opt=type=overlay,"
               f'volume-opt=device=overlay,"volume-opt=o=lowerdir={lower},upperdir={layer}/upper,workdir={layer}/work"')

    with tempfile.TemporaryDirectory(prefix="rab-moveit-") as folder:
        base = Path(folder)
        os.chmod(base, 0o755)
        with IsolatedProfile() as env:
            made = run([find_blender(), "--background", "--factory-startup", "--python-exit-code", "1", "--python",
                        str(ROOT / "tests" / "blender_billee_export.py"), "--", str(base), "--with-arm"], env)
        if made.returncode != 0:
            sys.exit("Blender export failed:\n" + made.stdout[-3000:] + made.stderr[-1000:])
        strip_render_sensor(base / "robot_description")
        shutil.copy(base / "robot_description_export.md", evidence / "robot_description_export.md")
        command = ["docker", "run", "--rm", "--init", "--network", "none", "--user", f"{os.getuid()}:{os.getgid()}",
                   "--entrypoint", "bash", "-v", f"{software}:{MOUNT}:ro", "--mount", overlay,
                   "-v", f"{ROOT}:/toolkit:ro", "-v", f"{base}:/cases:ro", "-v", f"{evidence}:/evidence",
                   find_image(), "/toolkit/tests/ros/run_moveit.sh"]
        process = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 encoding="utf-8", errors="replace", timeout=560, check=False)
    (evidence / "summary.log").write_text(process.stdout, encoding="utf-8")
    lines = [line for line in process.stdout.splitlines() if line.startswith("MOVEIT_")]
    for line in lines:
        print(line.replace("MOVEIT_", "", 1))
    passed = sum(1 for line in lines if line.startswith("MOVEIT_PASS"))
    failed = len(lines) - passed
    print(f"\n{passed} passed, {failed} failed")
    if process.returncode and not failed:
        print(process.stdout[-1500:])
    return 1 if failed or process.returncode else 0


if __name__ == "__main__":
    sys.exit(main())
