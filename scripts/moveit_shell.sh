#!/bin/bash
# Open a ROS 2 Humble shell with MoveIt and your exported packages, without changing billee-software-2027.
#
#   bash scripts/moveit_shell.sh <export folder>             # interactive shell
#   bash scripts/moveit_shell.sh <export folder> <command>   # run one command
#
# Run scripts/setup_moveit_env.sh once first. The shell joins the rover's ROS graph (domain 42).
set -eu

[ $# -ge 1 ] || { echo "Usage: bash scripts/moveit_shell.sh <export folder> [command...]" >&2; exit 2; }
EXPORT=$(CDPATH= cd -- "$1" && pwd)
shift
M=$(CDPATH= cd -- "${BILLEE_MOVEIT_ENV:-$HOME/.cache/billee-creation-toolkit/moveit-env}" && pwd)
REPO=$(CDPATH= cd -- "${BILLEE_SOFTWARE_DIR:-$HOME/billee-software-2027}" && pwd)
WS=/workspaces/billee-software-2027/ros2_ws
NETWORK=${BILLEE_NETWORK:-host}
DOMAIN=${ROS_DOMAIN_ID:-42}
IMAGE=${BILLEE_ROS_IMAGE:-$(docker images --format '{{.Repository}}:{{.Tag}}' | grep '^vsc-billee-software-2027' | grep -v -- '-uid:' | head -n 1)}
[ -d "$M/upper/conda-meta" ] || { echo "No MoveIt layer found. Run: bash scripts/setup_moveit_env.sh" >&2; exit 1; }
[ -d "$EXPORT/billee_arm_moveit_config" ] || { echo "No billee_arm_moveit_config in $EXPORT." >&2; exit 1; }

STARTUP='
PREFIX='"$WS"'/.pixi/envs/default
export CONDA_PREFIX=$PREFIX PATH=$PREFIX/bin:$PATH HOME=/tmp/moveit-home
mkdir -p "$HOME" /tmp/arm_ws/src
for script in "$PREFIX"/etc/conda/activate.d/*.sh; do
    case "$script" in *binutils*|*gcc*|*gfortran*|*gxx*) ;; *) . "$script" ;; esac
done
. '"$WS"'/install/local_setup.bash
for package in robot_description billee_arm_moveit_config; do
    [ -d "/export/$package" ] && cp -r "/export/$package" /tmp/arm_ws/src/
done
(cd /tmp/arm_ws && colcon build >/tmp/arm_ws/build.log 2>&1) || { tail -20 /tmp/arm_ws/build.log; exit 1; }
. /tmp/arm_ws/install/local_setup.bash
if [ $# -gt 0 ]; then exec "$@"; fi
echo "Ready. Start the rover first, then run: ros2 launch billee_arm_moveit_config moveit.launch.py"
exec bash --norc
'
TTY=()
[ -t 0 ] && TTY=(-it)
exec docker run --rm --init "${TTY[@]}" --network "$NETWORK" --user "$(id -u):$(id -g)" -e ROS_DOMAIN_ID="$DOMAIN" \
    --entrypoint bash -v "$REPO:/workspaces/billee-software-2027:ro" -v "$EXPORT:/export:ro" \
    --mount "type=volume,dst=$WS/.pixi/envs/default,volume-driver=local,volume-opt=type=overlay,volume-opt=device=overlay,\"volume-opt=o=lowerdir=$REPO/ros2_ws/.pixi/envs/default,upperdir=$M/upper,workdir=$M/work\"" \
    "$IMAGE" -c "$STARTUP" moveit-shell "$@"
