#!/bin/bash
# Build the MoveIt test layer used by scripts/moveit_check.py.
#
#   sh scripts/setup_moveit_env.sh
#
# MoveIt 2.5.9 is layered on a read-only view of billee-software-2027's pixi environment, with every
# installed package pinned to its current version. All writes go to BILLEE_MOVEIT_ENV, which defaults
# to ~/.cache/billee-creation-toolkit/moveit-env. billee-software-2027 is not changed.
set -eu

M=${BILLEE_MOVEIT_ENV:-$HOME/.cache/billee-creation-toolkit/moveit-env}
REPO=${BILLEE_SOFTWARE_DIR:-$HOME/billee-software-2027}
WS=/workspaces/billee-software-2027/ros2_ws
IMAGE=${BILLEE_ROS_IMAGE:-$(docker images --format '{{.Repository}}:{{.Tag}}' | grep '^vsc-billee-software-2027' | grep -v -- '-uid:' | head -n 1)}
[ -n "$IMAGE" ] || { echo "No billee-software-2027 dev container image found." >&2; exit 1; }
[ -x "$REPO/ros2_ws/.pixi/envs/default/bin/ros2" ] || { echo "No installed pixi environment in $REPO." >&2; exit 1; }

mkdir -p "$M"
M=$(CDPATH= cd -- "$M" && pwd)
# Overlay work folders are owned by root, so clear this layer through Docker.
docker run --rm --user 0:0 --entrypoint sh -v "$M:/moveit" "$IMAGE" -c 'rm -rf /moveit/upper /moveit/work /moveit/ws/.pixi'
mkdir -p "$M/ws/.pixi/envs/default" "$M/ws/src" "$M/ws/install" "$M/upper" "$M/work" "$M/cache" "$M/home"
cp "$REPO/ros2_ws/pixi.toml" "$REPO/ros2_ws/pixi.lock" "$M/ws/"

python3 -I - "$M/ws/pixi.toml" "$REPO/ros2_ws/.pixi/envs/default/conda-meta" <<'EOF'
import json, pathlib, re, sys

path, meta = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
text = path.read_text()
marker = "[feature.linux-cuda.dependencies]\n"
if marker not in text:
    raise SystemExit("default environment section not found")
packages = ["moveit-ros-move-group", "moveit-planners-ompl", "moveit-kinematics",
            "moveit-simple-controller-manager", "moveit-configs-utils"]
# 2.5.9 is the build compatible with the team's locked ROS packages, and matches their moveit2 fork.
text = text.replace(marker, marker + "".join(f'ros-humble-{n} = "2.5.9.*"\n' for n in packages), 1)

pins = {}
for item in sorted(meta.glob("*.json")):
    record = json.loads(item.read_text())
    pins[record["name"]] = record["version"]

# Test what is actually installed, so drop manifest entries that are not (in this copy only).
kept, dropped, section = [], [], None
for line in text.splitlines(True):
    heading = re.match(r"\s*\[([^\]]+)\]\s*$", line)
    if heading:
        section = heading.group(1)
    entry = re.match(r'"?([A-Za-z0-9_.-]+)"?\s*=', line)
    if section in ("dependencies", "feature.linux-cuda.dependencies") and entry:
        if entry.group(1) not in pins and not entry.group(1).startswith("ros-humble-moveit"):
            dropped.append(entry.group(1))
            continue
    kept.append(line)
text = "".join(kept)
table = "[feature.team-pins.dependencies]\n" + "".join(f'"{n}" = "=={v}"\n' for n, v in sorted(pins.items()))
old = 'default = { features = ["linux-cuda"]}'
if old not in text:
    raise SystemExit("default environment definition not found")
text = text.replace(old, 'default = { features = ["linux-cuda", "team-pins"]}', 1)
path.write_text(text.replace("[environments]\n", table + "\n[environments]\n", 1))
print(f"Pinned {len(pins)} packages. Not installed, so left out: {', '.join(dropped) or 'none'}")
EOF

docker run --rm --user "$(id -u):$(id -g)" --entrypoint bash \
    -e HOME=/moveit/home -e PIXI_CACHE_DIR=/moveit/cache -e RATTLER_CACHE_DIR=/moveit/cache \
    -v "$M:/moveit" -v "$M/ws:$WS" \
    -v "$REPO/ros2_ws/src:$WS/src:ro" -v "$REPO/ros2_ws/install:$WS/install:ro" \
    --mount "type=volume,dst=$WS/.pixi/envs/default,volume-driver=local,volume-opt=type=overlay,volume-opt=device=overlay,\"volume-opt=o=lowerdir=$REPO/ros2_ws/.pixi/envs/default,upperdir=$M/upper,workdir=$M/work\"" \
    "$IMAGE" -c "cd $WS && /opt/pixi/bin/pixi install -e default --color never"
echo "MoveIt layer ready in $M"
