#!/bin/sh
# Rover & Arm Builder installer for Linux and macOS.
#   Install:   sh Install-Linux.sh
#   Uninstall: sh Install-Linux.sh --uninstall
#   Choose a Blender: BLENDER=/path/to/blender sh Install-Linux.sh
set -u

PACKAGE=user_default.rover_arm_builder
MODULE=bl_ext.user_default.rover_arm_builder
MIN_MAJOR=5
MIN_MINOR=3

HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
ACTION=install
case "${1:-}" in
    --uninstall|uninstall) ACTION=uninstall ;;
    "") ;;
    *) echo "Usage: sh Install-Linux.sh [--uninstall]"; exit 2 ;;
esac

fail() {
    echo "" >&2
    echo "ERROR: $1" >&2
    exit 1
}

BEST=""
BEST_SCORE=-1
BEST_VERSION=""
USE_FLATPAK=0

version_of() {
    "$@" --version 2>/dev/null | sed -n 's/^Blender \([0-9][0-9]*\.[0-9][0-9]*\).*/\1/p' | head -n 1
}

consider() {
    [ -n "$1" ] && [ -x "$1" ] || return 0
    found=$(version_of "$1")
    [ -n "$found" ] || return 0
    score=$(( ${found%%.*} * 1000 + ${found#*.} ))
    if [ "$score" -gt "$BEST_SCORE" ]; then
        BEST=$1
        BEST_SCORE=$score
        BEST_VERSION=$found
        USE_FLATPAK=0
    fi
}

find_blender() {
    if [ -n "${BLENDER:-}" ]; then
        consider "$BLENDER"
        [ -n "$BEST" ] || fail "BLENDER is set to '$BLENDER', but that is not a working Blender."
        return
    fi
    consider "$(command -v blender 2>/dev/null || true)"
    consider /snap/bin/blender
    consider /Applications/Blender.app/Contents/MacOS/Blender
    consider "$HOME/Applications/Blender.app/Contents/MacOS/Blender"
    for candidate in /opt/blender*/blender "$HOME"/blender*/blender "$HOME"/Applications/blender*/blender; do
        consider "$candidate"
    done
    if command -v flatpak >/dev/null 2>&1 && flatpak info org.blender.Blender >/dev/null 2>&1; then
        found=$(version_of flatpak run org.blender.Blender)
        if [ -n "$found" ]; then
            score=$(( ${found%%.*} * 1000 + ${found#*.} ))
            if [ "$score" -gt "$BEST_SCORE" ]; then
                BEST=flatpak
                BEST_SCORE=$score
                BEST_VERSION=$found
                USE_FLATPAK=1
            fi
        fi
    fi
}

run_blender() {
    if [ "$USE_FLATPAK" -eq 1 ]; then
        flatpak run --filesystem="$HERE":ro --filesystem="$(dirname -- "${ZIP:-$HERE/x}")":ro org.blender.Blender "$@"
    else
        "$BEST" "$@"
    fi
}

find_zip() {
    ZIP=""
    for candidate in "$HERE"/rover_arm_builder-*.zip "$HERE"/../dist/rover_arm_builder-*.zip; do
        [ -f "$candidate" ] && ZIP=$candidate
    done
    [ -n "$ZIP" ] || fail "Could not find rover_arm_builder-*.zip next to this installer."
}

is_enabled() {
    run_blender --background --python-exit-code 1 --python-expr \
        "import bpy, sys; sys.exit(0 if '$MODULE' in bpy.context.preferences.addons else 1)" >/dev/null 2>&1
}

echo "Rover & Arm Builder"
echo "Looking for Blender $MIN_MAJOR.$MIN_MINOR or newer..."
find_blender
if [ -z "$BEST" ]; then
    fail "Blender was not found. Install it from https://www.blender.org/download/ and run this again."
fi
if [ "$BEST_SCORE" -lt $(( MIN_MAJOR * 1000 + MIN_MINOR )) ]; then
    fail "Found Blender $BEST_VERSION, but $MIN_MAJOR.$MIN_MINOR or newer is required."
fi
echo "Using Blender $BEST_VERSION ($BEST)"

if [ "$ACTION" = uninstall ]; then
    run_blender --command extension remove "$PACKAGE" || fail "Blender could not remove the extension."
    echo "Removed Rover & Arm Builder."
    exit 0
fi

find_zip
echo "Installing $(basename -- "$ZIP")..."
run_blender --command extension install-file -r user_default -e "$ZIP" || fail "Blender could not install the extension."
is_enabled || fail "The extension installed but is not enabled. Enable it in Edit > Preferences > Add-ons."

echo ""
echo "Installed and enabled."
echo "Open Blender, press N in the 3D Viewport, and choose the Robot tab."
