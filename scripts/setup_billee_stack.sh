#!/bin/bash
# Build a private test copy of the latest BILLEE stack with MoveIt 2 added.
#
#   bash scripts/setup_billee_stack.sh
#
# Clones billee-software-2027 into BILLEE_STACK_DIR (default ~/.cache/billee-creation-toolkit/stack),
# installs its locked packages plus MoveIt 2.5.9 on a read-only view of your existing pixi environment,
# and builds the rover and arm packages there. Your billee-software-2027 checkout is not changed.
set -eu

M=${BILLEE_STACK_DIR:-$HOME/.cache/billee-creation-toolkit/stack}
REPO=${BILLEE_SOFTWARE_DIR:-$HOME/billee-software-2027}
REF=${BILLEE_SOFTWARE_REF:-PreRelease-DashBoardDev}
URL=https://github.com/BILLEE-BroncoSTAR/billee-software-2027.git
MOUNT=/workspaces/billee-software-2027
LOWER=$REPO/ros2_ws/.pixi/envs/default
IMAGE=${BILLEE_ROS_IMAGE:-$(docker images --format '{{.Repository}}:{{.Tag}}' | grep '^vsc-billee-software-2027' | grep -v -- '-uid:' | head -n 1)}
[ -n "$IMAGE" ] || { echo "No billee-software-2027 dev container image found." >&2; exit 1; }
[ -x "$LOWER/bin/ros2" ] || { echo "No installed pixi environment in $REPO." >&2; exit 1; }

mkdir -p "$M"
M=$(CDPATH= cd -- "$M" && pwd)
mkdir -p "$M/upper" "$M/work" "$M/cache" "$M/home"
# An existing copy is rebuilt as it is. Use an empty BILLEE_STACK_DIR to test a newer commit.
if [ ! -e "$M/sw/.git" ]; then
git clone -q --depth 1 --branch "$REF" "$URL" "$M/sw"
git -C "$M/sw" submodule update -q --init --depth 1 third_party/JetsonGPIO
mkdir -p "$M/sw/ros2_ws/.pixi/envs/default"

python3 -I - "$M/sw/ros2_ws" <<'EOF'
import pathlib, re, sys

root = pathlib.Path(sys.argv[1])
pins, inside, platform = {}, False, None
for line in (root / "pixi.lock").read_text().splitlines():
    if re.match(r"^  [a-z0-9_-]+:$", line):
        inside = line.strip() == "default:"
    elif inside and re.match(r"^      p\d+:$", line):
        platform = line.strip()
    elif inside and platform == "p1:":
        found = re.match(r"\s+- conda: \S+/([^/]+?)(?:\.conda|\.tar\.bz2)$", line)
        if found:
            name, version, _build = found.group(1).rsplit("-", 2)
            pins[name] = version
if len(pins) < 100:
    raise SystemExit("Could not read the default environment from pixi.lock")

path = root / "pixi.toml"
text = path.read_text()
marker = "[feature.linux-cuda.dependencies]\n"
old = 'default = { features = ["linux-cuda"]}'
if marker not in text or old not in text:
    raise SystemExit("Unexpected pixi.toml layout")
packages = ["moveit-ros-move-group", "moveit-planners-ompl", "moveit-kinematics",
            "moveit-simple-controller-manager", "moveit-configs-utils"]
# 2.5.9 is the build compatible with the locked ROS packages and with the team's moveit2 fork.
text = text.replace(marker, marker + "".join(f'ros-humble-{n} = "2.5.9.*"\n' for n in packages), 1)
text = text.replace(old, 'default = { features = ["linux-cuda", "team-pins"]}', 1)
table = "[feature.team-pins.dependencies]\n" + "".join(f'"{n}" = "=={v}"\n' for n, v in sorted(pins.items()))
path.write_text(text.replace("[environments]\n", table + "\n[environments]\n", 1))
print(f"Pinned {len(pins)} locked packages")
EOF
fi

docker run --rm --user "$(id -u):$(id -g)" --entrypoint bash \
    -e HOME=/stack/home -e PIXI_CACHE_DIR=/stack/cache -e RATTLER_CACHE_DIR=/stack/cache \
    -v "$M:/stack" -v "$M/sw:$MOUNT" \
    --mount "type=volume,dst=$MOUNT/ros2_ws/.pixi/envs/default,volume-driver=local,volume-opt=type=overlay,volume-opt=device=overlay,\"volume-opt=o=lowerdir=$LOWER,upperdir=$M/upper,workdir=$M/work\"" \
    "$IMAGE" -c "
set -e
cd $MOUNT/ros2_ws
/opt/pixi/bin/pixi install -e default --color never
PREFIX=$MOUNT/ros2_ws/.pixi/envs/default
export CONDA_PREFIX=\$PREFIX PATH=\$PREFIX/bin:\$PATH
set +eu
for script in \$PREFIX/etc/conda/activate.d/*.sh; do . \"\$script\"; done
set -e
# pxx includes a roboclaw header that needs JetsonGPIO.h, which that package does not export.
export CPATH=$MOUNT/third_party/JetsonGPIO/include:$MOUNT/ros2_ws/build/roboclaw_hardware_interface/jetson_gpio
export LD_LIBRARY_PATH=$MOUNT/ros2_ws/build/roboclaw_hardware_interface/jetson_gpio:\${LD_LIBRARY_PATH:-}
colcon build --parallel-workers \$(nproc) --event-handlers console_cohesion+ --base-paths src \
    --packages-up-to chassis_bringup arm_bringup arm_description robot_description teleop navigation status_light arm_vision zed2i people camera_stream \
    --cmake-args ' -DCMAKE_BUILD_TYPE=Release' ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' ' -DCMAKE_CXX_FLAGS=\"-Wl,--allow-shlib-undefined\"' \
    > /stack/build.log 2>&1 || { tail -30 /stack/build.log; exit 1; }
"
echo "BILLEE stack ready at $(git -C "$M/sw" rev-parse --short HEAD) in $M"
