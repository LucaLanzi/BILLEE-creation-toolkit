"""Run every local check and print an honest summary.

Run from any folder: python scripts/check.py
ROS, MoveIt, and Gazebo gates are separate and are reported as not run here.
"""

import platform
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

from common import ROOT, IsolatedProfile, build_zip, find_blender, run

RESULTS = []


def record(name, process):
    passed = process.returncode == 0
    RESULTS.append((name, passed))
    print(f"[{'PASS' if passed else 'FAIL'}] {name}")
    if not passed:
        print((process.stdout + process.stderr)[-4000:])
    return passed


def blender_script(blender, env, script, *args):
    command = [blender, "--background", "--factory-startup", "--python-exit-code", "1", "--python", str(script)]
    if args:
        command += ["--", *args]
    return run(command, env, timeout=300)


def extension_enabled(blender, env):
    expr = ("import bpy, sys; "
            "sys.exit(0 if 'bl_ext.user_default.rover_arm_builder' in bpy.context.preferences.addons else 7)")
    return run([blender, "--background", "--python-expr", expr], env).returncode == 0


def run_installer(blender, env, archive):
    """Install from the real bundle, in a path with spaces, through this platform's script."""
    from package_release import make_bundle

    with tempfile.TemporaryDirectory(prefix="rab installer ") as folder:
        base = Path(folder)
        bundle = make_bundle(archive, base / "bundle output")
        unpacked = base / "my downloads" / "Rover Arm Builder"
        with zipfile.ZipFile(bundle) as source:
            source.extractall(unpacked)
            modes = {i.filename: (i.external_attr >> 16) & 0o111 for i in source.infolist()}
        names = {"Install-Linux.sh", "Install-macOS.command", "Install-Windows.bat", "README.md", "LICENSE", archive.name}
        RESULTS.append(("Installer bundle contents", names == set(modes)))
        print(f"[{'PASS' if names == set(modes) else 'FAIL'}] Installer bundle contents")
        unix_ok = bool(modes.get("Install-Linux.sh")) and bool(modes.get("Install-macOS.command"))
        RESULTS.append(("Installer executable bits", unix_ok))
        print(f"[{'PASS' if unix_ok else 'FAIL'}] Installer executable bits")

        script_env = dict(env, BLENDER=blender, RAB_NO_PAUSE="1")
        if platform.system() == "Windows":
            command = ["cmd", "/c", str(unpacked / "Install-Windows.bat")]
            uninstall = "uninstall"
        else:
            command = ["sh", str(unpacked / ("Install-macOS.command" if platform.system() == "Darwin" else "Install-Linux.sh"))]
            uninstall = "--uninstall"
        if record("One-step install", run(command, script_env)):
            record("Install verified enabled", _status(extension_enabled(blender, env)))
            record("One-step reinstall over existing", run(command, script_env))
            record("One-step uninstall", run([*command, uninstall], script_env))
            record("Uninstall verified disabled", _status(not extension_enabled(blender, env)))


def _status(passed):
    return subprocess.CompletedProcess([], 0 if passed else 1, "", "state check failed")


def run_gui(blender, env):
    """Open a real window using the isolated profile. Screenshots go to dist/gui-evidence."""
    prefs = "import bpy; bpy.context.preferences.view.show_splash = False; bpy.ops.wm.save_userpref()"
    saved = run([blender, "--background", "--factory-startup", "--python-exit-code", "1", "--python-expr", prefs], env)
    if not record("GUI profile setup", saved):
        return
    with tempfile.TemporaryDirectory(prefix="rab gui ") as folder:
        made = blender_script(blender, env, ROOT / "examples" / "generate_demo.py", folder)
        if not record("GUI demo scene", made):
            return
        evidence = ROOT / "dist" / "gui-evidence"
        process = run([blender, "--window-geometry", "0", "0", "1600", "1000", str(Path(folder) / "demo_rover_arm.blend"),
                       "--python", str(ROOT / "tests" / "blender_gui.py"), "--", str(evidence)], env, timeout=120)
        for line in process.stdout.splitlines():
            if line.startswith(("GUI_RESULT", "GUI_MISSING", "GUI_DRAW_ERROR")):
                print("   ", line)
        if "Traceback" in process.stdout + process.stderr or "Timeout (" in process.stderr:
            process.returncode = process.returncode or 1
        record("Windowed interface test", process)


def main():
    record("Core and exporter unit tests",
           run([sys.executable, "-B", "-m", "unittest", "discover", "-s", "tests", "-t", "."]))
    blender = find_blender()
    with IsolatedProfile() as env:
        record("Headless Blender tests", blender_script(blender, env, ROOT / "tests" / "blender_headless.py"))
        try:
            archive = build_zip(blender, env)
            RESULTS.append(("Extension build and ZIP validation", True))
            print("[PASS] Extension build and ZIP validation")
        except SystemExit as exc:
            RESULTS.append(("Extension build and ZIP validation", False))
            print(f"[FAIL] Extension build and ZIP validation\n{exc}")
            archive = None
        if archive is not None:
            installed = run([blender, "--command", "extension", "install-file", "-r", "user_default", str(archive)], env)
            if record("Extension install", installed):
                record("Enable, disable, re-enable", blender_script(blender, env, ROOT / "tests" / "blender_lifecycle.py"))
                record("Extension uninstall",
                       run([blender, "--command", "extension", "remove", "user_default.rover_arm_builder"], env))
                run_installer(blender, env, archive)
        with tempfile.TemporaryDirectory(prefix="rab demo ") as folder:
            record("Demo generation in a path with spaces",
                   blender_script(blender, env, ROOT / "examples" / "generate_demo.py", folder))

        if "--gui" in sys.argv:
            run_gui(blender, env)

    print("\nROS 2, xacro, TF, and Gazebo checks are separate: python scripts/ros_gates.py")
    failed = [name for name, passed in RESULTS if not passed]
    print(f"{len(RESULTS) - len(failed)} of {len(RESULTS)} checks passed.")
    return 1 if failed else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except subprocess.TimeoutExpired as exc:
        sys.exit(f"Timed out: {exc.cmd}")
