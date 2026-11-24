"""Prove a generated robot_description can replace the real one with no software changes.

Run from any folder: python scripts/dropin_check.py

It exports the BILLEE rover through Blender, then runs the unmodified rover launch files
against the stock package and the generated one. billee-software-2027 is mounted read-only,
and the container has no network. The ZED render sensor is removed from temporary copies
only, because this check has no display. Its presence is checked in the full model's SDF.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from common import ROOT, IsolatedProfile, find_blender, run
from ros_gates import MOUNT, find_image


def strip_render_sensor(package: Path) -> None:
    path = package / "urdf" / "robot_gz.urdf.xacro"
    text = path.read_text(encoding="utf-8")
    text = re.sub(r'<sensor name="zed2i".*?</sensor>', "", text, flags=re.S)
    text = re.sub(r'<plugin name="ignition::gazebo::systems::Sensors".*?</plugin>', "", text, flags=re.S)
    path.write_text(text, encoding="utf-8")


def prepare(case: Path, source: Path) -> None:
    shutil.copytree(source, case / "full" / "robot_description", ignore=shutil.ignore_patterns("__pycache__"))
    shutil.copytree(source, case / "sim" / "robot_description", ignore=shutil.ignore_patterns("__pycache__"))
    strip_render_sensor(case / "sim" / "robot_description")


def control_block(path: Path) -> list[tuple]:
    root = ET.parse(path).getroot().find("ros2_control")
    return [(e.tag, tuple(sorted(e.attrib.items())), (e.text or "").strip()) for e in root.iter()]


def parse(output: str) -> dict[str, str]:
    return dict(line.split(" ", 1)[1].split("=", 1) for line in output.splitlines() if line.startswith("RESULT "))


def main() -> int:
    software = Path(os.environ.get("BILLEE_SOFTWARE_DIR", Path.home() / "billee-software-2027")).resolve()
    stock = software / "ros2_ws" / "src" / "robot_description"
    if not stock.is_dir():
        sys.exit(f"No robot_description found in {software}.")
    image = find_image()
    evidence = ROOT / "dist" / "dropin-check"
    shutil.rmtree(evidence, ignore_errors=True)
    evidence.mkdir(parents=True)
    outputs = {}

    with tempfile.TemporaryDirectory(prefix="rab-dropin-") as folder:
        base = Path(folder)
        os.chmod(base, 0o755)
        export = base / "export"
        with IsolatedProfile() as env:
            made = run([find_blender(), "--background", "--factory-startup", "--python-exit-code", "1", "--python",
                        str(ROOT / "tests" / "blender_billee_export.py"), "--", str(export)], env)
        if made.returncode != 0:
            sys.exit("Blender export failed:\n" + made.stdout[-3000:] + made.stderr[-1000:])
        for line in made.stdout.splitlines():
            if line.startswith("DROPIN_STATUS"):
                print(line.split(" ", 1)[1])
        shutil.copy(export / "robot_description_export.md", evidence / "robot_description_export.md")
        prepare(base / "stock", stock)
        prepare(base / "dropin", export / "robot_description")

        for case in ("stock", "dropin"):
            print(f"Running the unmodified rover launch with the {case} model...", flush=True)
            command = ["docker", "run", "--rm", "--init", "--network", "none", "--user", f"{os.getuid()}:{os.getgid()}",
                       "--entrypoint", "bash", "-v", f"{software}:{MOUNT}:ro", "-v", f"{ROOT}:/toolkit:ro",
                       "-v", f"{base}:/cases:ro", "-v", f"{evidence}:/evidence", image,
                       "/toolkit/tests/ros/run_dropin.sh", case]
            process = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     encoding="utf-8", errors="replace", timeout=260, check=False)
            outputs[case] = process.stdout
            (evidence / f"{case}.summary.log").write_text(process.stdout, encoding="utf-8")

    stock_result, dropin_result = parse(outputs["stock"]), parse(outputs["dropin"])
    failures = []
    print(f"\n{'Check':42} {'Stock':28} Generated")
    for key in sorted(set(stock_result) | set(dropin_result)):
        a, b = stock_result.get(key, "missing"), dropin_result.get(key, "missing")
        same = a == b
        wanted = b != "False" and b != "missing" and b != "none"
        if not (same and wanted):
            failures.append(key)
        print(f"{key:42} {a:28} {b}{'' if same and wanted else '   <-- FAIL'}")
    for case in ("stock", "dropin"):
        metrics = [l.split(" ", 1)[1] for l in outputs[case].splitlines() if l.startswith("METRIC ")]
        print(f"{case} measured: {', '.join(metrics) or 'none'}")

    try:
        same_control = control_block(evidence / "stock.real.urdf") == control_block(evidence / "dropin.real.urdf")
    except (OSError, ET.ParseError, AttributeError):
        same_control = False
    print(f"Real-hardware ros2_control block, including CAN IDs, identical: {same_control}")
    if not same_control:
        failures.append("ros2_control block")
    print("\nPASS: drop-in matches the stock model" if not failures else f"\nFAIL: {', '.join(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
