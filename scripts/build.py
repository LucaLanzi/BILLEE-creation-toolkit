"""Build and validate the installable extension ZIP.

Run from any folder: python scripts/build.py
"""

import hashlib

from common import IsolatedProfile, build_zip, find_blender

if __name__ == "__main__":
    blender = find_blender()
    with IsolatedProfile() as env:
        archive = build_zip(blender, env)
    print(f"Built {archive}")
    print(f"SHA-256 {hashlib.sha256(archive.read_bytes()).hexdigest()}")
