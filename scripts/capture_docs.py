"""Regenerate the README screenshots in docs/images from the real Blender interface.

Run from any folder: python scripts/capture_docs.py
It opens a Blender window briefly, using an isolated profile, then closes it.
"""

import sys
import tempfile
from pathlib import Path

from common import ROOT, IsolatedProfile, find_blender, run


def main() -> int:
    blender = find_blender()
    prefs = "import bpy; bpy.context.preferences.view.show_splash = False; bpy.ops.wm.save_userpref()"
    with IsolatedProfile() as env, tempfile.TemporaryDirectory(prefix="rab docs ") as folder:
        saved = run([blender, "--background", "--factory-startup", "--python-exit-code", "1", "--python-expr", prefs], env)
        made = run([blender, "--background", "--factory-startup", "--python-exit-code", "1", "--python",
                    str(ROOT / "examples" / "generate_demo.py"), "--", folder], env)
        if saved.returncode or made.returncode:
            sys.exit("Could not prepare the demo:\n" + made.stdout[-2000:] + made.stderr[-1000:])
        process = run([blender, "--enable-event-simulate", "--window-geometry", "0", "0", "1600", "1000",
                       str(Path(folder) / "demo_rover_arm.blend"), "--python", str(ROOT / "tests" / "blender_capture.py"),
                       "--", str(ROOT / "docs" / "images")], env, timeout=130)
    for line in process.stdout.splitlines():
        if line.startswith("CAPTURE_"):
            print(line)
    if process.returncode:
        print((process.stdout + process.stderr)[-2500:])
    return process.returncode


if __name__ == "__main__":
    sys.exit(main())
