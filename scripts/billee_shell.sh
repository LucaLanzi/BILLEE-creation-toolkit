#!/bin/bash
# Open a ROS 2 shell with MoveIt and your exported packages, without changing billee-software-2027.
#
#   bash scripts/billee_shell.sh <export folder>             # interactive shell
#   bash scripts/billee_shell.sh <export folder> <command>   # run one command
#
# Run scripts/setup_billee_stack.sh once first. The shell joins the rover's ROS graph (domain 42).
set -eu

[ $# -ge 1 ] || { echo "Usage: bash scripts/billee_shell.sh <export folder> [command...]" >&2; exit 2; }
EXPORT=$(CDPATH= cd -- "$1" && pwd)
shift
M=$(CDPATH= cd -- "${BILLEE_STACK_DIR:-$HOME/.cache/billee-creation-toolkit/stack}" && pwd)
REPO=$(CDPATH= cd -- "${BILLEE_SOFTWARE_DIR:-$HOME/billee-software-2027}" && pwd)
MOUNT=/workspaces/billee-software-2027
WS=$MOUNT/ros2_ws
NETWORK=${BILLEE_NETWORK:-host}
DOMAIN=${ROS_DOMAIN_ID:-42}
IMAGE=${BILLEE_ROS_IMAGE:-$(docker images --format '{{.Repository}}:{{.Tag}}' | grep '^vsc-billee-software-2027' | grep -v -- '-uid:' | head -n 1)}
[ -f "$M/sw/ros2_ws/install/setup.bash" ] || { echo "No stack found. Run: bash scripts/setup_billee_stack.sh" >&2; exit 1; }
[ -d "$EXPORT/billee_arm_moveit_config" ] || { echo "No billee_arm_moveit_config in $EXPORT." >&2; exit 1; }

STARTUP='
PREFIX='"$WS"'/.pixi/envs/default
export CONDA_PREFIX=$PREFIX PATH=$PREFIX/bin:$PATH HOME=/tmp/billee-home
export LD_LIBRARY_PATH='"$WS"'/build/roboclaw_hardware_interface/jetson_gpio:${LD_LIBRARY_PATH:-}
mkdir -p "$HOME" /tmp/billee_ws/src
for script in "$PREFIX"/etc/conda/activate.d/*.sh; do
    case "$script" in *binutils*|*gcc*|*gfortran*|*gxx*) ;; *) . "$script" ;; esac
done
. '"$WS"'/install/local_setup.bash
for package in robot_description arm_description billee_arm_moveit_config; do
    [ -d "/export/$package" ] && cp -r "/export/$package" /tmp/billee_ws/src/
done
(cd /tmp/billee_ws && colcon build >/tmp/billee_ws/build.log 2>&1) || { tail -20 /tmp/billee_ws/build.log; exit 1; }
. /tmp/billee_ws/install/local_setup.bash
if [ $# -gt 0 ]; then exec "$@"; fi
echo "Ready. Start the rover and arm, then run: ros2 launch billee_arm_moveit_config moveit.launch.py"
exec bash --norc
'
TTY=()
[ -t 0 ] && TTY=(-it)
exec docker run --rm --init "${TTY[@]}" --network "$NETWORK" --user "$(id -u):$(id -g)" -e ROS_DOMAIN_ID="$DOMAIN" \
    --entrypoint bash -v "$M/sw:$MOUNT:ro" -v "$EXPORT:/export:ro" \
    --mount "type=volume,dst=$WS/.pixi/envs/default,volume-driver=local,volume-opt=type=overlay,volume-opt=device=overlay,\"volume-opt=o=lowerdir=$REPO/ros2_ws/.pixi/envs/default,upperdir=$M/upper,workdir=$M/work\"" \
    "$IMAGE" -c "$STARTUP" billee-shell "$@"
