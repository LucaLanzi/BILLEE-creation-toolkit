"""Build the one-download installer bundle.

Run from any folder: python scripts/package_release.py
Output: dist/BILLEE-Creation-Toolkit-<version>-Installer.zip
"""

from __future__ import annotations

import hashlib
import stat
import tomllib
import zipfile
from pathlib import Path

from common import DIST, ROOT, SOURCE, IsolatedProfile, build_zip, find_blender

FIXED_TIME = (2026, 1, 1, 0, 0, 0)


def _add(bundle: zipfile.ZipFile, name: str, data: bytes, executable: bool = False) -> None:
    info = zipfile.ZipInfo(name, FIXED_TIME)
    info.compress_type = zipfile.ZIP_DEFLATED
    info.create_system = 3
    mode = 0o755 if executable else 0o644
    info.external_attr = (stat.S_IFREG | mode) << 16
    bundle.writestr(info, data)


def make_bundle(extension_zip: Path, output_dir: Path = DIST) -> Path:
    version = tomllib.loads((SOURCE / "blender_manifest.toml").read_text(encoding="utf-8"))["version"]
    output_dir.mkdir(parents=True, exist_ok=True)
    target = output_dir / f"BILLEE-Creation-Toolkit-{version}-Installer.zip"
    installers = ROOT / "installers"
    windows = (installers / "Install-Windows.bat").read_bytes().replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")
    with zipfile.ZipFile(target, "w") as bundle:
        _add(bundle, "Install-Linux.sh", (installers / "Install-Linux.sh").read_bytes(), True)
        _add(bundle, "Install-macOS.command", (installers / "Install-macOS.command").read_bytes(), True)
        _add(bundle, "Install-Windows.bat", windows)
        _add(bundle, "README.md", (ROOT / "README.md").read_bytes())
        _add(bundle, "LICENSE", (ROOT / "LICENSE").read_bytes())
        _add(bundle, extension_zip.name, extension_zip.read_bytes())
    return target


if __name__ == "__main__":
    blender = find_blender()
    with IsolatedProfile() as env:
        archive = build_zip(blender, env)
    result = make_bundle(archive)
    print(f"Built {result}")
    print(f"SHA-256 {hashlib.sha256(result.read_bytes()).hexdigest()}")
