"""Test exports through the real BILLEE launch files, using the dashboard's own commands.

    python scripts/billee_check.py [all|rover|arm]

Needs scripts/setup_billee_stack.sh. Everything runs in an offline container against a private copy
of billee-software-2027. The ZED render sensor is removed from temporary copies because this check
has no display.
"""

from __future__ import annotations

import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from common import ROOT, IsolatedProfile, find_blender, run
from ros_gates import MOUNT, find_image

DASHBOARD = "https://github.com/LucaLanzi/BILLEE-dashboard-tool.git"


def strip_render_sensor(package: Path) -> None:
    path = package / "urdf" / "robot_gz.urdf.xacro"
    text = path.read_text(encoding="utf-8")
    text = re.sub(r'<sensor name="zed2i".*?</sensor>', "", text, flags=re.S)
    text = re.sub(r'<plugin name="ignition::gazebo::systems::Sensors".*?</plugin>', "", text, flags=re.S)
    path.write_text(text, encoding="utf-8")


def dashboard_commands(stack: Path, folder: Path) -> tuple[str, str]:
    """Ask the dashboard pinned by this stack which rover arguments it launches."""
    pinned = subprocess.run(["git", "-C", str(stack / "sw"), "ls-tree", "HEAD", "BILLEE-dashboard-tool"],
                            capture_output=True, text=True, check=True).stdout.split()[2]
    target = folder / "dashboard"
    subprocess.run(["git", "clone", "-q", DASHBOARD, str(target)], check=True)
    subprocess.run(["git", "-C", str(target), "checkout", "-q", pinned], check=True)
    code = (
        "import json, sys\n"
        "sys.path.insert(0, sys.argv[1])\n"
        "from billee_dash import modes\n"
        "def walk(v):\n"
        "    if isinstance(v, dict):\n"
        "        if isinstance(v.get('cmd'), str): yield v['cmd']\n"
        "        for x in v.values(): yield from walk(x)\n"
        "    elif isinstance(v, (list, tuple)):\n"
        "        for x in v: yield from walk(x)\n"
        "def rover(mode, options):\n"
        "    return next(c for c in walk(modes.build(mode, options)) if 'rover.launch.py' in c)\n"
        "print(json.dumps([rover('teleop-drive', {'drivetrain': 'sim', 'camera': 'none'}),\n"
        "                  rover('teleop-arm', {'drivetrain': 'sim', 'camera': 'none', 'arm_hw': 'mock'})]))\n"
    )
    import json

    output = subprocess.run([sys.executable, "-I", "-c", code, str(target)], capture_output=True, text=True, check=True).stdout
    drive, arm = json.loads(output)

    def args(command):
        return " ".join(a for a in shlex.split(command.split("rover.launch.py", 1)[1]) if not a.startswith("status_light:="))

    return args(drive), args(arm)


def main() -> int:
    stage = sys.argv[1] if len(sys.argv) > 1 else "all"
    software = Path(os.environ.get("BILLEE_SOFTWARE_DIR", Path.home() / "billee-software-2027")).resolve()
    default = Path.home() / ".cache" / "billee-creation-toolkit" / "stack"
    stack = Path(os.environ.get("BILLEE_STACK_DIR", default)).resolve()
    if not (stack / "sw" / "ros2_ws" / "install" / "setup.bash").is_file():
        sys.exit(f"No BILLEE stack in {stack}. Run: bash scripts/setup_billee_stack.sh")
    evidence = ROOT / "dist" / "billee-check"
    evidence.mkdir(parents=True, exist_ok=True)
    lower = software / "ros2_ws" / ".pixi" / "envs" / "default"
    overlay = (f"type=volume,dst={MOUNT}/ros2_ws/.pixi/envs/default,volume-driver=local,volume-opt=type=overlay,"
               f'volume-opt=device=overlay,"volume-opt=o=lowerdir={lower},upperdir={stack}/upper,workdir={stack}/work"')

    with tempfile.TemporaryDirectory(prefix="rab-billee-") as folder:
        base = Path(folder)
        os.chmod(base, 0o755)
        drive, arm = dashboard_commands(stack, base)
        print(f"Dashboard rover args: drive [{drive}], arm [{arm}]", flush=True)
        (base / "rover_args.txt").write_text(drive)
        (base / "arm_rover_args.txt").write_text(arm)
        with IsolatedProfile() as env:
            env = dict(env, BILLEE_SOFTWARE_DIR=str(stack / "sw"))
            made = run([find_blender(), "--background", "--factory-startup", "--python-exit-code", "1", "--python",
                        str(ROOT / "tests" / "blender_billee_export.py"), "--", str(base)], env, timeout=120)
        if made.returncode != 0:
            sys.exit("Blender export failed:\n" + made.stdout[-3000:] + made.stderr[-1000:])
        shutil.copytree(stack / "sw" / "ros2_ws" / "src" / "robot_description", base / "stock" / "robot_description")
        strip_render_sensor(base / "stock" / "robot_description")
        strip_render_sensor(base / "rover" / "robot_description")
        strip_render_sensor(base / "assembly" / "robot_description")
        for name in ("arm_export.md", "robot_description_export.md"):
            shutil.copy(base / "assembly" / name, evidence / name)
        command = ["docker", "run", "--rm", "--init", "--network", "none", "--user", f"{os.getuid()}:{os.getgid()}",
                   "--entrypoint", "bash", "-v", f"{stack / 'sw'}:{MOUNT}:ro", "--mount", overlay,
                   "-v", f"{ROOT}:/toolkit:ro", "-v", f"{base}:/cases:ro", "-v", f"{evidence}:/evidence",
                   find_image(), "/toolkit/tests/ros/run_billee.sh", stage]
        process = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 encoding="utf-8", errors="replace", timeout=900, check=False)
    (evidence / f"summary-{stage}.log").write_text(process.stdout, encoding="utf-8")

    failures = 0
    lines = process.stdout.splitlines()
    stock = dict(l[6:].split("=", 1) for l in lines if l.startswith("STOCK "))
    rover = dict(l[6:].split("=", 1) for l in lines if l.startswith("ROVER "))
    if stock or rover:
        print(f"\n{'Rover check':42} {'Stock':10} Generated")
        for key in sorted(set(stock) | set(rover)):
            a, b = stock.get(key, "missing"), rover.get(key, "missing")
            good = a == b and b not in ("False", "missing", "none")
            failures += not good
            print(f"{key:42} {a[:9]:10} {b[:44]}{'' if good else '   <-- FAIL'}")
    checks = [l for l in lines if l.startswith("CHECK_")]
    for line in checks:
        print(line.replace("CHECK_", "", 1)[:190])
    failed = sum(1 for l in checks if l.startswith("CHECK_FAIL"))
    print(f"\n{sum(1 for l in checks if l.startswith('CHECK_PASS'))} passed, {failed + failures} failed")
    if process.returncode and not (failed or failures):
        print(process.stdout[-1500:])
    return 1 if failed or failures or process.returncode else 0


if __name__ == "__main__":
    sys.exit(main())
