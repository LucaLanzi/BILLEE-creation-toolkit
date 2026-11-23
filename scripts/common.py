"""Shared helpers for the build and check scripts. Works on Linux, macOS, and Windows."""

from __future__ import annotations

import glob
import os
import platform
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "rover_arm_builder"
DIST = ROOT / "dist"


def find_blender() -> str:
    """Use $BLENDER, then PATH, then each platform's standard install location."""
    configured = os.environ.get("BLENDER")
    if configured:
        return configured
    found = shutil.which("blender")
    if found:
        return found
    system = platform.system()
    if system == "Darwin":
        candidates = ["/Applications/Blender.app/Contents/MacOS/Blender"]
    elif system == "Windows":
        base = os.environ.get("ProgramFiles", r"C:\Program Files")
        candidates = sorted(glob.glob(os.path.join(base, "Blender Foundation", "Blender *", "blender.exe")), reverse=True)
    else:
        candidates = ["/snap/bin/blender"]
    for candidate in candidates:
        if Path(candidate).exists():
            return candidate
    sys.exit("Blender was not found. Set the BLENDER environment variable to its executable.")


class IsolatedProfile:
    """Temporary Blender user folders, so checks never touch the real configuration."""

    def __enter__(self) -> dict[str, str]:
        self._tmp = tempfile.TemporaryDirectory(prefix="rab profile ")
        base = Path(self._tmp.name)
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
        for key, name in (("BLENDER_USER_CONFIG", "config"), ("BLENDER_USER_SCRIPTS", "scripts"),
                          ("BLENDER_USER_DATAFILES", "datafiles"), ("BLENDER_USER_EXTENSIONS", "extensions")):
            path = base / name
            path.mkdir()
            env[key] = str(path)
        return env

    def __exit__(self, *exc) -> None:
        self._tmp.cleanup()


def run(args: list[str], env: dict[str, str] | None = None, timeout: int = 300, cwd: Path = ROOT) -> subprocess.CompletedProcess:
    return subprocess.run(args, cwd=cwd, env=env, timeout=timeout, capture_output=True, text=True,
                          encoding="utf-8", errors="replace", check=False)


def build_zip(blender: str, env: dict[str, str]) -> Path:
    DIST.mkdir(exist_ok=True)
    validated = run([blender, "--command", "extension", "validate", str(SOURCE)], env)
    if validated.returncode != 0:
        sys.exit("Manifest validation failed:\n" + validated.stdout + validated.stderr)
    built = run([blender, "--command", "extension", "build", "--source-dir", str(SOURCE), "--output-dir", str(DIST)], env)
    if built.returncode != 0:
        sys.exit("Extension build failed:\n" + built.stdout + built.stderr)
    archives = sorted(DIST.glob("rover_arm_builder-*.zip"), key=lambda p: p.stat().st_mtime)
    if not archives:
        sys.exit("The build finished but produced no ZIP.\n" + built.stdout)
    checked = run([blender, "--command", "extension", "validate", str(archives[-1])], env)
    if checked.returncode != 0:
        sys.exit("ZIP validation failed:\n" + checked.stdout + checked.stderr)
    return archives[-1]
