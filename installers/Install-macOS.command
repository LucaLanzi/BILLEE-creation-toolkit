#!/bin/sh
# Double-click to install Rover & Arm Builder on an Apple Silicon Mac.
# To uninstall, run in Terminal: sh Install-macOS.command --uninstall
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
sh "$HERE/Install-Linux.sh" "$@"
STATUS=$?
if [ -t 0 ]; then
    printf '\nPress Return to close this window.'
    read -r _
fi
exit "$STATUS"
